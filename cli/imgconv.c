/*
 * imgconv.c — Image to Genesis VDP tile converter
 *
 * Loads any image (PNG, JPEG, BMP, TGA, GIF, PSD, HDR, PIC), resizes to
 * 320×224, quantizes to Genesis 9-bit color space (3 bits per channel, 512
 * possible colors), selects optimal colors, assigns to 4 palettes of 16,
 * packs into 4bpp tile data with per-tile palette selection.
 *
 * Output: preview PNG showing Genesis-accurate rendering, and/or binary
 * frame for TCP push to the Mandelbrot receiver ROM.
 *
 * Usage:
 *   imgconv input [-o preview.png] [-b frame.bin]
 *   imgconv input -H host [-p port] [-d mode]
 *
 * No external dependencies — uses stb_image/stb_image_write (bundled).
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <getopt.h>
#include <math.h>

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include <signal.h>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <sys/time.h>

/* ── Graceful shutdown flag ── */
static volatile sig_atomic_t g_stop = 0;
static void sigint_handler(int sig) { (void)sig; g_stop = 1; }

/* ── Genesis display geometry ── */
#define SCREEN_W      320
#define SCREEN_H      224
#define TILE_W        8
#define TILE_H        8
#define TILES_X       (SCREEN_W / TILE_W)   /* 40 */
#define TILES_Y       (SCREEN_H / TILE_H)   /* 28 */
#define NUM_TILES     (TILES_X * TILES_Y)   /* 1120 */
#define TILE_BYTES    32                     /* 8×8 × 4bpp */
#define RAW_SIZE      (NUM_TILES * TILE_BYTES)  /* 35840 */

#define NUM_PALETTES  4
#define COLORS_PER_PAL 16
#define USABLE_PER_PAL 15   /* index 0 = transparent/backdrop on Genesis */
#define USABLE_COLORS  (NUM_PALETTES * USABLE_PER_PAL)  /* 60 */
#define MAX_DUAL_TILES 128  /* Genesis RAM limited — 128 tiles fits budget */

/* ── Frame header (same as mandelbrot.c) ── */
#define FRAME_MAGIC   0x4D42

/* ── Genesis 9-bit color ── */
typedef struct { uint8_t r, g, b; } gen_color_t;  /* 0-7 each */

/* ── Timing ── */
static int64_t usec_now(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (int64_t)tv.tv_sec * 1000000 + tv.tv_usec;
}

/* ── Quantize 8-bit channel to Genesis 3-bit ── */
static uint8_t to_gen(uint8_t v)
{
    /* Round to nearest of 8 levels: 0,36,73,109,146,182,219,255 */
    return (uint8_t)((v * 7 + 128) / 255);
}

/* ── Genesis 3-bit back to 8-bit for display ── */
static uint8_t from_gen(uint8_t v)
{
    return (uint8_t)((v * 255 + 3) / 7);
}

/* ── Color distance (squared, in 8-bit space) ── */
static int color_dist_sq(gen_color_t a, gen_color_t b)
{
    int dr = from_gen(a.r) - from_gen(b.r);
    int dg = from_gen(a.g) - from_gen(b.g);
    int db = from_gen(a.b) - from_gen(b.b);
    return dr*dr + dg*dg + db*db;
}

/* ── Pack gen_color_t to a unique 9-bit key ── */
static uint16_t gen_key(gen_color_t c)
{
    return ((uint16_t)c.r << 6) | ((uint16_t)c.g << 3) | c.b;
}

/* ══════════════════════════════════════════════════════════════════
 *  Image loading (any format supported by stb_image)
 * ══════════════════════════════════════════════════════════════════ */

static uint8_t *load_image(const char *path, int *w, int *h)
{
    int channels;
    uint8_t *pixels = stbi_load(path, w, h, &channels, 3);  /* force RGB */
    if (!pixels) {
        fprintf(stderr, "%s: %s\n", path, stbi_failure_reason());
        return NULL;
    }
    return pixels;
}

/* ══════════════════════════════════════════════════════════════════
 *  Bilinear resize (RGB)
 * ══════════════════════════════════════════════════════════════════ */

static uint8_t *resize_rgb(const uint8_t *src, int sw, int sh,
                            int dw, int dh)
{
    uint8_t *dst = malloc(dw * dh * 3);

    for (int dy = 0; dy < dh; dy++) {
        double sy = (dy + 0.5) * sh / (double)dh - 0.5;
        int y0 = (int)floor(sy);
        int y1 = y0 + 1;
        double fy = sy - y0;
        if (y0 < 0) y0 = 0;
        if (y1 >= sh) y1 = sh - 1;

        for (int dx = 0; dx < dw; dx++) {
            double sx = (dx + 0.5) * sw / (double)dw - 0.5;
            int x0 = (int)floor(sx);
            int x1 = x0 + 1;
            double fx = sx - x0;
            if (x0 < 0) x0 = 0;
            if (x1 >= sw) x1 = sw - 1;

            for (int c = 0; c < 3; c++) {
                double v = (1-fx)*(1-fy) * src[(y0*sw+x0)*3+c]
                         + (  fx)*(1-fy) * src[(y0*sw+x1)*3+c]
                         + (1-fx)*(  fy) * src[(y1*sw+x0)*3+c]
                         + (  fx)*(  fy) * src[(y1*sw+x1)*3+c];
                dst[(dy*dw+dx)*3+c] = (uint8_t)(v + 0.5);
            }
        }
    }
    return dst;
}

/* ══════════════════════════════════════════════════════════════════
 *  Color selection: pick top N unique 9-bit colors by frequency.
 *  When unique count <= N, just return all of them.
 * ══════════════════════════════════════════════════════════════════ */

typedef struct {
    gen_color_t color;
    int freq;
} color_freq_t;

static int freq_cmp_desc(const void *a, const void *b)
{
    return ((const color_freq_t *)b)->freq - ((const color_freq_t *)a)->freq;
}

/*
 * Select top n_colors unique colors from the image by pixel frequency.
 * Returns actual number of unique colors found (<= n_colors).
 */
static int select_top_colors(const gen_color_t *pixels, int n_pixels,
                              gen_color_t *palette, int n_colors)
{
    /* Histogram over 9-bit color space (512 entries) */
    int freq[512] = {0};
    for (int i = 0; i < n_pixels; i++)
        freq[gen_key(pixels[i])]++;

    /* Collect unique colors with frequencies */
    color_freq_t unique[512];
    int n_unique = 0;
    for (int k = 0; k < 512; k++) {
        if (freq[k] > 0) {
            unique[n_unique].color.r = (k >> 6) & 7;
            unique[n_unique].color.g = (k >> 3) & 7;
            unique[n_unique].color.b = k & 7;
            unique[n_unique].freq = freq[k];
            n_unique++;
        }
    }

    /* Sort by frequency descending, take top n_colors */
    qsort(unique, n_unique, sizeof(color_freq_t), freq_cmp_desc);

    int count = n_unique < n_colors ? n_unique : n_colors;
    for (int i = 0; i < count; i++)
        palette[i] = unique[i].color;

    return count;
}

/* Forward declaration — used by tile_error_exact before definition */
static uint8_t nearest_in_pal(gen_color_t pixel, gen_color_t *pal,
                               int n_colors);

/* ══════════════════════════════════════════════════════════════════
 *  Palette assignment with iterative refinement
 *
 *  1. Initial split: luminance-sorted 4×16
 *  2. Assign each tile to best palette (padded sampling)
 *  3. Rebuild each palette from its assigned tiles' pixels
 *  4. Repeat 2-3 for several iterations
 * ══════════════════════════════════════════════════════════════════ */

static int luma_cmp(const void *a, const void *b)
{
    const gen_color_t *ca = a, *cb = b;
    int la = ca->r * 2 + ca->g * 5 + ca->b;
    int lb = cb->r * 2 + cb->g * 5 + cb->b;
    return la - lb;
}

/* Compute palette error for a tile's pixels */
static int tile_error_exact(gen_color_t *img, int tx, int ty,
                              gen_color_t *pal, int n_colors)
{
    int err = 0;
    for (int py = 0; py < TILE_H; py++) {
        int sy = ty * TILE_H + py;
        for (int px = 0; px < TILE_W; px++) {
            int sx = tx * TILE_W + px;
            gen_color_t pixel = img[sy * SCREEN_W + sx];
            uint8_t ci = nearest_in_pal(pixel, pal, n_colors);
            err += color_dist_sq(pixel, pal[ci]);
        }
    }
    return err;
}

/* Assign each tile to its best palette using padded error */
static void assign_tiles(gen_color_t *img, gen_color_t pal[4][16],
                           uint8_t *pal_map)
{
    for (int ty = 0; ty < TILES_Y; ty++) {
        for (int tx = 0; tx < TILES_X; tx++) {
            int idx = ty * TILES_X + tx;
            int best = 0;
            int best_err = tile_error_exact(img, tx, ty, pal[0], 16);
            for (int p = 1; p < NUM_PALETTES; p++) {
                int e = tile_error_exact(img, tx, ty, pal[p], 16);
                if (e < best_err) { best_err = e; best = p; }
            }
            pal_map[idx] = (uint8_t)best;
        }
    }
}

/*
 * K-means tile clustering: group tiles by average color, then build
 * each palette from its cluster's pixels.  This ensures tiles with
 * similar color needs (e.g. skin tones) share a palette that spans
 * their full color range.
 */

/* Compute average color of a tile */
static void tile_avg(gen_color_t *img, int tx, int ty, float *ar, float *ag, float *ab)
{
    int sr = 0, sg = 0, sb = 0;
    for (int py = 0; py < TILE_H; py++) {
        int sy = ty * TILE_H + py;
        for (int px = 0; px < TILE_W; px++) {
            int sx = tx * TILE_W + px;
            gen_color_t c = img[sy * SCREEN_W + sx];
            sr += c.r; sg += c.g; sb += c.b;
        }
    }
    *ar = sr / 64.0f;
    *ag = sg / 64.0f;
    *ab = sb / 64.0f;
}

/* Build palette for a cluster: slot 0 = black (backdrop/transparent),
 * slots 1-15 selected by frequency × diversity.
 *
 * Pure frequency picks miss rare but visually important colors (e.g.
 * bright yellow in a mandrill eye surrounded by green fur).  We use a
 * greedy approach: first pick = most frequent color, each subsequent
 * pick maximizes  freq_weight * min_distance_to_already_selected.
 * This ensures both common AND outlier colors get representation. */
static void build_cluster_palette(gen_color_t *img, const uint8_t *assignments,
                                    int cluster, gen_color_t *out_pal)
{
    int freq[512] = {0};
    for (int ty = 0; ty < TILES_Y; ty++) {
        for (int tx = 0; tx < TILES_X; tx++) {
            if (assignments[ty * TILES_X + tx] != cluster) continue;
            for (int py = 0; py < TILE_H; py++) {
                int sy = ty * TILE_H + py;
                for (int px = 0; px < TILE_W; px++) {
                    int sx = tx * TILE_W + px;
                    freq[gen_key(img[sy * SCREEN_W + sx])]++;
                }
            }
        }
    }

    color_freq_t cands[512];
    int n = 0;
    for (int k = 0; k < 512; k++) {
        if (freq[k] > 0) {
            cands[n].color.r = (k >> 6) & 7;
            cands[n].color.g = (k >> 3) & 7;
            cands[n].color.b = k & 7;
            cands[n].freq = freq[k];
            n++;
        }
    }

    /* Slot 0: black (transparent/backdrop on Genesis VDP) */
    out_pal[0].r = 0; out_pal[0].g = 0; out_pal[0].b = 0;

    if (n == 0) {
        for (int i = 1; i < COLORS_PER_PAL; i++) out_pal[i] = out_pal[0];
        return;
    }

    int count = n < USABLE_PER_PAL ? n : USABLE_PER_PAL;

    /* Track which candidates are already selected and min dist to selected */
    int *min_dist = malloc(n * sizeof(int));
    int *selected = calloc(n, sizeof(int));

    /* First pick: most frequent color */
    int best = 0;
    for (int i = 1; i < n; i++)
        if (cands[i].freq > cands[best].freq) best = i;
    selected[best] = 1;
    out_pal[1] = cands[best].color;

    /* Init min_dist from the first selected color */
    for (int i = 0; i < n; i++)
        min_dist[i] = color_dist_sq(cands[i].color, out_pal[1]);

    /* Greedy picks 2..count: maximize  sqrt(freq) * min_dist */
    for (int pick = 1; pick < count; pick++) {
        float best_score = -1;
        int best_idx = 0;
        for (int i = 0; i < n; i++) {
            if (selected[i]) continue;
            float fw = sqrtf((float)cands[i].freq);
            float score = fw * min_dist[i];
            if (score > best_score) { best_score = score; best_idx = i; }
        }
        selected[best_idx] = 1;
        out_pal[pick + 1] = cands[best_idx].color;

        /* Update min_dist for remaining candidates */
        for (int i = 0; i < n; i++) {
            if (selected[i]) continue;
            int d = color_dist_sq(cands[i].color, cands[best_idx].color);
            if (d < min_dist[i]) min_dist[i] = d;
        }
    }

    /* Fill unused slots (if fewer unique colors than 15) */
    for (int i = count; i < USABLE_PER_PAL; i++)
        out_pal[i + 1] = out_pal[(i % count) + 1];

    free(min_dist);
    free(selected);
}

static void refine_palettes(gen_color_t *img, gen_color_t *all64,
                              int n_colors,
                              gen_color_t pal[4][16], uint8_t *pal_map,
                              int iterations)
{
    (void)all64;
    (void)n_colors;

    /* Compute average color for each tile and flag dead (all-black) tiles.
     * Dead tiles (pillarbox/letterbox borders) are excluded from k-means
     * so they don't waste an entire cluster on black. */
    float tile_r[NUM_TILES], tile_g[NUM_TILES], tile_b[NUM_TILES];
    uint8_t dead[NUM_TILES];
    int n_live = 0;
    for (int ty = 0; ty < TILES_Y; ty++)
        for (int tx = 0; tx < TILES_X; tx++) {
            int idx = ty * TILES_X + tx;
            tile_avg(img, tx, ty, &tile_r[idx], &tile_g[idx], &tile_b[idx]);
            dead[idx] = (tile_r[idx] == 0.0f && tile_g[idx] == 0.0f
                         && tile_b[idx] == 0.0f) ? 1 : 0;
            if (!dead[idx]) n_live++;
        }

    /* K-means: 4 clusters, initialize with spread centroids.
     * Seed only from live (non-black) tiles. */
    float cr[4], cg[4], cb[4];
    {
        float luma[NUM_TILES];
        int order[NUM_TILES];
        int n_ord = 0;
        for (int i = 0; i < NUM_TILES; i++) {
            if (dead[i]) continue;
            luma[n_ord] = tile_r[i] * 2 + tile_g[i] * 5 + tile_b[i];
            order[n_ord] = i;
            n_ord++;
        }
        /* Simple sort by luminance for seeding */
        for (int i = 0; i < n_ord - 1; i++)
            for (int j = i + 1; j < n_ord; j++)
                if (luma[j] < luma[i]) {
                    float lt = luma[i]; luma[i] = luma[j]; luma[j] = lt;
                    int tmp = order[i]; order[i] = order[j]; order[j] = tmp;
                }
        for (int k = 0; k < 4; k++) {
            int seed = order[(2 * k + 1) * n_ord / 8];
            cr[k] = tile_r[seed];
            cg[k] = tile_g[seed];
            cb[k] = tile_b[seed];
        }
    }

    /* K-means iterations — skip dead tiles */
    for (int iter = 0; iter < iterations; iter++) {
        /* Assign live tiles to nearest centroid */
        for (int i = 0; i < NUM_TILES; i++) {
            if (dead[i]) { pal_map[i] = 0; continue; }
            float best_d = 1e9f;
            int best_k = 0;
            for (int k = 0; k < 4; k++) {
                float dr = tile_r[i] - cr[k];
                float dg = tile_g[i] - cg[k];
                float db = tile_b[i] - cb[k];
                float d = dr*dr + dg*dg + db*db;
                if (d < best_d) { best_d = d; best_k = k; }
            }
            pal_map[i] = (uint8_t)best_k;
        }

        /* Recompute centroids from live tiles only */
        float sr[4]={0}, sg[4]={0}, sb[4]={0};
        int cnt[4] = {0};
        for (int i = 0; i < NUM_TILES; i++) {
            if (dead[i]) continue;
            int k = pal_map[i];
            sr[k] += tile_r[i];
            sg[k] += tile_g[i];
            sb[k] += tile_b[i];
            cnt[k]++;
        }
        for (int k = 0; k < 4; k++) {
            if (cnt[k] > 0) {
                cr[k] = sr[k] / cnt[k];
                cg[k] = sg[k] / cnt[k];
                cb[k] = sb[k] / cnt[k];
            }
        }
    }

    /* Build palettes from clusters, then refine with error-based
     * reassignment so tiles migrate to the palette that actually
     * serves their pixels best (not just their centroid). */
    for (int refine = 0; refine < 3; refine++) {
        for (int k = 0; k < 4; k++)
            build_cluster_palette(img, pal_map, k, pal[k]);
        assign_tiles(img, pal, pal_map);
    }

    /* One final palette build from the converged assignments */
    for (int k = 0; k < 4; k++)
        build_cluster_palette(img, pal_map, k, pal[k]);

    /* ── Deduplicate colors across palettes ──
     *
     * Colors that appear in multiple palettes waste slots.  For each
     * duplicate, keep it in the palette where it has the highest pixel
     * frequency and replace the other copy with the best unused color
     * from that cluster (highest freq color not already in any palette).
     */
    {
        /* Build per-cluster frequency tables */
        int cluster_freq[4][512];
        memset(cluster_freq, 0, sizeof(cluster_freq));
        for (int ty = 0; ty < TILES_Y; ty++)
            for (int tx = 0; tx < TILES_X; tx++) {
                int k = pal_map[ty * TILES_X + tx];
                for (int py = 0; py < TILE_H; py++) {
                    int sy = ty * TILE_H + py;
                    for (int px = 0; px < TILE_W; px++) {
                        int sx = tx * TILE_W + px;
                        cluster_freq[k][gen_key(img[sy * SCREEN_W + sx])]++;
                    }
                }
            }

        /* For each 9-bit color, find which palettes contain it */
        for (int key = 0; key < 512; key++) {
            /* Collect palette slots that have this color */
            int owners[4] = {-1, -1, -1, -1};
            int n_owners = 0;
            for (int k = 0; k < 4; k++)
                for (int c = 1; c < COLORS_PER_PAL; c++) {
                    int ck = gen_key(pal[k][c]);
                    if (ck == key) { owners[n_owners++] = k; break; }
                }
            if (n_owners <= 1) continue;

            /* Keep in palette with highest frequency for this color */
            int best_k = owners[0];
            int best_f = cluster_freq[owners[0]][key];
            for (int i = 1; i < n_owners; i++) {
                int f = cluster_freq[owners[i]][key];
                if (f > best_f) { best_f = f; best_k = owners[i]; }
            }

            /* Replace duplicate slots in other palettes, but only if
             * the color is much less important there than in the winner.
             * This prevents stripping colors that multiple palettes
             * genuinely need (e.g. skin tones shared across clusters). */
            for (int i = 0; i < n_owners; i++) {
                int k = owners[i];
                if (k == best_k) continue;

                /* Skip dedup if this color is important to palette k —
                 * keep it if its frequency is >= 20% of the winner's */
                int my_f = cluster_freq[k][key];
                if (best_f > 0 && my_f >= best_f / 5) continue;

                /* Find the slot */
                int slot = -1;
                for (int c = 1; c < COLORS_PER_PAL; c++)
                    if (gen_key(pal[k][c]) == key) { slot = c; break; }
                if (slot < 0) continue;

                /* Collect colors already in ANY palette */
                uint8_t used[512] = {0};
                for (int pk = 0; pk < 4; pk++)
                    for (int c = 1; c < COLORS_PER_PAL; c++)
                        used[gen_key(pal[pk][c])] = 1;

                /* Find best replacement: highest freq unused color
                 * from this cluster */
                int best_rep = -1, best_rep_f = 0;
                for (int ck = 0; ck < 512; ck++) {
                    if (used[ck] || ck == 0) continue;
                    if (cluster_freq[k][ck] > best_rep_f) {
                        best_rep_f = cluster_freq[k][ck];
                        best_rep = ck;
                    }
                }

                if (best_rep >= 0) {
                    pal[k][slot].r = (best_rep >> 6) & 7;
                    pal[k][slot].g = (best_rep >> 3) & 7;
                    pal[k][slot].b = best_rep & 7;
                } else {
                    /* No unused colors left — set to black */
                    pal[k][slot].r = 0;
                    pal[k][slot].g = 0;
                    pal[k][slot].b = 0;
                }
            }
        }

        /* Final reassignment with deduplicated palettes */
        assign_tiles(img, pal, pal_map);
    }
}

/* ══════════════════════════════════════════════════════════════════
 *  Ordered dithering (8×8 Bayer matrix)
 * ══════════════════════════════════════════════════════════════════ */

/* 8×8 Bayer threshold map, values 0..63.
 * Normalized: (bayer[y][x] / 64.0 - 0.5) * spread */
static const uint8_t bayer8[8][8] = {
    {  0, 32,  8, 40,  2, 34, 10, 42 },
    { 48, 16, 56, 24, 50, 18, 58, 26 },
    { 12, 44,  4, 36, 14, 46,  6, 38 },
    { 60, 28, 52, 20, 62, 30, 54, 22 },
    {  3, 35, 11, 43,  1, 33,  9, 41 },
    { 51, 19, 59, 27, 49, 17, 57, 25 },
    { 15, 47,  7, 39, 13, 45,  5, 37 },
    { 63, 31, 55, 23, 61, 29, 53, 21 },
};

/* Dither spread in 8-bit space.  One Genesis color step = 255/7 ≈ 36.
 * Using ~half a step gives good results without excessive noise. */
#define DITHER_SPREAD  28

/* Apply Bayer dither offset to a pixel (in 8-bit space), return
 * the dithered gen_color_t (still clamped to 0-7 per channel). */
static gen_color_t dither_pixel(gen_color_t pixel, int x, int y)
{
    /* Threshold in range [-SPREAD, +SPREAD] */
    int thr = (int)bayer8[y & 7][x & 7] * (2 * DITHER_SPREAD) / 63
              - DITHER_SPREAD;

    int r8 = from_gen(pixel.r) + thr;
    int g8 = from_gen(pixel.g) + thr;
    int b8 = from_gen(pixel.b) + thr;

    if (r8 < 0) r8 = 0; else if (r8 > 255) r8 = 255;
    if (g8 < 0) g8 = 0; else if (g8 > 255) g8 = 255;
    if (b8 < 0) b8 = 0; else if (b8 > 255) b8 = 255;

    gen_color_t out;
    out.r = to_gen((uint8_t)r8);
    out.g = to_gen((uint8_t)g8);
    out.b = to_gen((uint8_t)b8);
    return out;
}

/* ══════════════════════════════════════════════════════════════════
 *  Dither mode selection
 * ══════════════════════════════════════════════════════════════════ */

enum dither_mode { DITHER_NONE, DITHER_BAYER, DITHER_FS };
static enum dither_mode g_dither = DITHER_FS;  /* default */
static int g_dual_layer = 0;  /* -2 flag: use BG_A + BG_B */

/* ══════════════════════════════════════════════════════════════════
 *  Per-tile palette selection + pixel mapping
 * ══════════════════════════════════════════════════════════════════ */

/* Find nearest color index in a palette.
 * Skips index 0 — on Genesis VDP, pixel value 0 is always transparent
 * (shows backdrop color), so we only use indices 1-15. */
static uint8_t nearest_in_pal(gen_color_t pixel, gen_color_t *pal,
                               int n_colors)
{
    /* Index 0 is transparent/backdrop — use it for black pixels */
    if (pixel.r == 0 && pixel.g == 0 && pixel.b == 0)
        return 0;
    int best = 1, best_d = color_dist_sq(pixel, pal[1]);
    for (int i = 2; i < n_colors; i++) {
        int d = color_dist_sq(pixel, pal[i]);
        if (d < best_d) { best_d = d; best = i; }
    }
    return (uint8_t)best;
}

/* Find nearest color across TWO palettes.  Returns palette index (0 or 1
 * relative to the pair) and color index within that palette via pointers. */
static void nearest_in_dual(gen_color_t pixel,
                             gen_color_t *pal_a, gen_color_t *pal_b,
                             int *out_layer, uint8_t *out_ci)
{
    if (pixel.r == 0 && pixel.g == 0 && pixel.b == 0) {
        *out_layer = 0; *out_ci = 0; return;
    }
    uint8_t ci_a = nearest_in_pal(pixel, pal_a, COLORS_PER_PAL);
    uint8_t ci_b = nearest_in_pal(pixel, pal_b, COLORS_PER_PAL);
    int d_a = color_dist_sq(pixel, pal_a[ci_a]);
    int d_b = color_dist_sq(pixel, pal_b[ci_b]);
    if (d_a <= d_b) { *out_layer = 0; *out_ci = ci_a; }
    else            { *out_layer = 1; *out_ci = ci_b; }
}

/* Compute tile error when pixel can pick from EITHER of two palettes */
static int tile_error_dual(gen_color_t *img, int tx, int ty,
                            gen_color_t *pal_a, gen_color_t *pal_b)
{
    int err = 0;
    for (int py = 0; py < TILE_H; py++) {
        int sy = ty * TILE_H + py;
        for (int px = 0; px < TILE_W; px++) {
            int sx = tx * TILE_W + px;
            gen_color_t pixel = img[sy * SCREEN_W + sx];
            int layer; uint8_t ci;
            nearest_in_dual(pixel, pal_a, pal_b, &layer, &ci);
            gen_color_t got = (layer == 0) ? pal_a[ci] : pal_b[ci];
            err += color_dist_sq(pixel, got);
        }
    }
    return err;
}


/*
 * Map a single pixel to its palette index, applying the current
 * dither mode.  For Bayer, the pixel is dithered before lookup.
 * For none/FS, the pixel is used as-is (FS error applied externally).
 */
static uint8_t map_pixel(gen_color_t pixel, gen_color_t *pal, int x, int y)
{
    if (g_dither == DITHER_BAYER) {
        gen_color_t d = dither_pixel(pixel, x, y);
        return nearest_in_pal(d, pal, COLORS_PER_PAL);
    }
    return nearest_in_pal(pixel, pal, COLORS_PER_PAL);
}

/*
 * Floyd-Steinberg error diffusion: process image in scanline order,
 * find nearest palette color, diffuse quantization error to neighbors.
 * Fills pixel_idx[] (palette index per pixel, scanline order).
 */
static void dither_floyd_steinberg(gen_color_t *img, gen_color_t pal[4][16],
                                     const uint8_t *pal_map, uint8_t *pixel_idx)
{
    int n = SCREEN_W * SCREEN_H;
    /* Error buffer in 8-bit space (signed) */
    int16_t *err_r = calloc(n, sizeof(int16_t));
    int16_t *err_g = calloc(n, sizeof(int16_t));
    int16_t *err_b = calloc(n, sizeof(int16_t));

    for (int y = 0; y < SCREEN_H; y++) {
        for (int x = 0; x < SCREEN_W; x++) {
            int i = y * SCREEN_W + x;
            int tx = x / TILE_W, ty = y / TILE_H;
            int p = pal_map[ty * TILES_X + tx];

            /* Current pixel + accumulated error */
            int r8 = from_gen(img[i].r) + err_r[i];
            int g8 = from_gen(img[i].g) + err_g[i];
            int b8 = from_gen(img[i].b) + err_b[i];
            if (r8 < 0) r8 = 0; else if (r8 > 255) r8 = 255;
            if (g8 < 0) g8 = 0; else if (g8 > 255) g8 = 255;
            if (b8 < 0) b8 = 0; else if (b8 > 255) b8 = 255;

            /* Quantize to nearest palette color */
            gen_color_t want;
            want.r = to_gen((uint8_t)r8);
            want.g = to_gen((uint8_t)g8);
            want.b = to_gen((uint8_t)b8);
            uint8_t ci = nearest_in_pal(want, pal[p], COLORS_PER_PAL);
            pixel_idx[i] = ci;

            /* Compute error in 8-bit space */
            gen_color_t got = pal[p][ci];
            int er = r8 - from_gen(got.r);
            int eg = g8 - from_gen(got.g);
            int eb = b8 - from_gen(got.b);

            /* Diffuse: right 7/16, below-left 3/16, below 5/16, below-right 1/16 */
            #define DIFFUSE(dx, dy, num) do { \
                int nx = x + (dx), ny = y + (dy); \
                if (nx >= 0 && nx < SCREEN_W && ny < SCREEN_H) { \
                    int ni = ny * SCREEN_W + nx; \
                    err_r[ni] += (int16_t)(er * (num) / 16); \
                    err_g[ni] += (int16_t)(eg * (num) / 16); \
                    err_b[ni] += (int16_t)(eb * (num) / 16); \
                } \
            } while(0)

            DIFFUSE( 1, 0, 7);
            DIFFUSE(-1, 1, 3);
            DIFFUSE( 0, 1, 5);
            DIFFUSE( 1, 1, 1);
            #undef DIFFUSE
        }
    }

    free(err_r); free(err_g); free(err_b);
}

/*
 * Process entire image: map pixels using pre-assigned pal_map.
 * Fills tiles[] (4bpp packed).
 */
static void process_tiles(gen_color_t *img, gen_color_t pal[4][16],
                           uint8_t *tiles, const uint8_t *pal_map)
{
    if (g_dither == DITHER_FS) {
        /* Floyd-Steinberg: compute all pixel indices in scanline order,
         * then pack into tile-order 4bpp */
        uint8_t *pixel_idx = malloc(SCREEN_W * SCREEN_H);
        dither_floyd_steinberg(img, pal, pal_map, pixel_idx);

        uint8_t *out = tiles;
        for (int ty = 0; ty < TILES_Y; ty++) {
            for (int tx = 0; tx < TILES_X; tx++) {
                for (int py = 0; py < TILE_H; py++) {
                    int sy = ty * TILE_H + py;
                    for (int px = 0; px < TILE_W; px += 2) {
                        int sx = tx * TILE_W + px;
                        uint8_t hi = pixel_idx[sy * SCREEN_W + sx];
                        uint8_t lo = pixel_idx[sy * SCREEN_W + sx + 1];
                        *out++ = (hi << 4) | lo;
                    }
                }
            }
        }
        free(pixel_idx);
    } else {
        /* Bayer or None: per-pixel mapping in tile order */
        uint8_t *out = tiles;
        for (int ty = 0; ty < TILES_Y; ty++) {
            for (int tx = 0; tx < TILES_X; tx++) {
                int tile_idx = ty * TILES_X + tx;
                int best_pal = pal_map[tile_idx];
                for (int py = 0; py < TILE_H; py++) {
                    int sy = ty * TILE_H + py;
                    for (int px = 0; px < TILE_W; px += 2) {
                        int sx = tx * TILE_W + px;
                        uint8_t hi = map_pixel(img[sy*SCREEN_W+sx],
                                                pal[best_pal], sx, sy);
                        uint8_t lo = map_pixel(img[sy*SCREEN_W+sx+1],
                                                pal[best_pal], sx+1, sy);
                        *out++ = (hi << 4) | lo;
                    }
                }
            }
        }
    }
}

/* ══════════════════════════════════════════════════════════════════
 *  Dual-layer tile processing (BG_A + BG_B)
 *
 *  For each tile, decide the best palette pair and which pixels go
 *  on which layer.  BG_A pixel = index 0 means transparent → BG_B
 *  shows through, and vice versa.
 * ══════════════════════════════════════════════════════════════════ */

typedef struct {
    uint16_t tile_idx;   /* position in 1120-tile grid */
    uint8_t  pal_a;      /* palette for BG_A */
    uint8_t  pal_b;      /* palette for BG_B */
    int      benefit;    /* error reduction from dual vs single */
} dual_info_t;

static int dual_cmp_desc(const void *a, const void *b)
{
    return ((const dual_info_t *)b)->benefit -
           ((const dual_info_t *)a)->benefit;
}

/*
 * For each tile, find the best single palette and the best palette pair.
 * Returns the number of tiles that benefit from dual-layer (up to max_dual).
 * Fills dual[] with sorted entries (best benefit first).
 */
static int compute_dual_info(gen_color_t *img, gen_color_t pal[4][16],
                              const uint8_t *pal_map, dual_info_t *dual,
                              int max_dual)
{
    dual_info_t all[NUM_TILES];
    int n = 0;

    for (int i = 0; i < NUM_TILES; i++) {
        int tx = i % TILES_X, ty = i / TILES_X;
        int best_pal = pal_map[i];
        int single_err = tile_error_exact(img, tx, ty, pal[best_pal], 16);
        if (single_err == 0) continue;  /* perfect already */

        /* Try all palette pairs — find the pair that minimizes error */
        int best_a = best_pal, best_b = best_pal;
        int best_dual_err = single_err;
        for (int pa = 0; pa < NUM_PALETTES; pa++) {
            for (int pb = pa + 1; pb < NUM_PALETTES; pb++) {
                int d = tile_error_dual(img, tx, ty, pal[pa], pal[pb]);
                if (d < best_dual_err) {
                    best_dual_err = d;
                    best_a = pa;
                    best_b = pb;
                }
            }
        }

        int benefit = single_err - best_dual_err;
        if (benefit > 0) {
            all[n].tile_idx = (uint16_t)i;
            all[n].pal_a = (uint8_t)best_a;
            all[n].pal_b = (uint8_t)best_b;
            all[n].benefit = benefit;
            n++;
        }
    }

    /* Sort by benefit descending, take top max_dual */
    qsort(all, n, sizeof(dual_info_t), dual_cmp_desc);
    int count = n < max_dual ? n : max_dual;
    memcpy(dual, all, count * sizeof(dual_info_t));
    return count;
}

/*
 * Floyd-Steinberg dithering aware of dual-layer tiles.
 * pixel_idx[]: color index per pixel (scanline order)
 * layer_map[]: 0=BG_A, 1=BG_B per pixel (scanline order)
 * pal_map_a[], pal_map_b[]: palette index per tile for each layer.
 * is_dual[]: 1 if tile has a BG_B layer, 0 otherwise.
 */
static void dither_fs_dual(gen_color_t *img, gen_color_t pal[4][16],
                            const uint8_t *pal_map_a, const uint8_t *pal_map_b,
                            const uint8_t *is_dual,
                            uint8_t *pixel_idx, uint8_t *layer_map)
{
    int n = SCREEN_W * SCREEN_H;
    int16_t *err_r = calloc(n, sizeof(int16_t));
    int16_t *err_g = calloc(n, sizeof(int16_t));
    int16_t *err_b = calloc(n, sizeof(int16_t));

    for (int y = 0; y < SCREEN_H; y++) {
        for (int x = 0; x < SCREEN_W; x++) {
            int i = y * SCREEN_W + x;
            int tx = x / TILE_W, ty = y / TILE_H;
            int tidx = ty * TILES_X + tx;
            int pa = pal_map_a[tidx];

            /* Current pixel + accumulated error */
            int r8 = from_gen(img[i].r) + err_r[i];
            int g8 = from_gen(img[i].g) + err_g[i];
            int b8 = from_gen(img[i].b) + err_b[i];
            if (r8 < 0) r8 = 0; else if (r8 > 255) r8 = 255;
            if (g8 < 0) g8 = 0; else if (g8 > 255) g8 = 255;
            if (b8 < 0) b8 = 0; else if (b8 > 255) b8 = 255;

            gen_color_t want;
            want.r = to_gen((uint8_t)r8);
            want.g = to_gen((uint8_t)g8);
            want.b = to_gen((uint8_t)b8);

            uint8_t ci;
            int layer = 0;
            if (is_dual[tidx]) {
                int pb = pal_map_b[tidx];
                nearest_in_dual(want, pal[pa], pal[pb], &layer, &ci);
            } else {
                ci = nearest_in_pal(want, pal[pa], COLORS_PER_PAL);
            }
            pixel_idx[i] = ci;
            layer_map[i] = (uint8_t)layer;

            /* Compute error */
            gen_color_t got;
            if (ci == 0) { got.r = 0; got.g = 0; got.b = 0; }
            else got = (layer == 0) ? pal[pa][ci] : pal[pal_map_b[tidx]][ci];
            int er = r8 - from_gen(got.r);
            int eg = g8 - from_gen(got.g);
            int eb = b8 - from_gen(got.b);

            /* Diffuse */
            #define DIFFUSE_D(dx, dy, num) do { \
                int nx = x + (dx), ny = y + (dy); \
                if (nx >= 0 && nx < SCREEN_W && ny < SCREEN_H) { \
                    int ni = ny * SCREEN_W + nx; \
                    err_r[ni] += (int16_t)(er * (num) / 16); \
                    err_g[ni] += (int16_t)(eg * (num) / 16); \
                    err_b[ni] += (int16_t)(eb * (num) / 16); \
                } \
            } while (0)
            DIFFUSE_D(1, 0, 7);
            DIFFUSE_D(-1, 1, 3);
            DIFFUSE_D(0, 1, 5);
            DIFFUSE_D(1, 1, 1);
            #undef DIFFUSE_D
        }
    }
    free(err_r); free(err_g); free(err_b);
}

/*
 * Process tiles in dual-layer mode.
 * tiles_a[]: BG_A tile data (always NUM_TILES tiles)
 * tiles_b[]: BG_B tile data (dual_count tiles, packed sequentially)
 */
static void process_tiles_dual(gen_color_t *img, gen_color_t pal[4][16],
                                uint8_t *tiles_a, uint8_t *tiles_b,
                                const uint8_t *pal_map_a,
                                const uint8_t *pal_map_b,
                                const uint8_t *is_dual,
                                const dual_info_t *dual_list, int dual_count)
{
    uint8_t *pixel_idx = malloc(SCREEN_W * SCREEN_H);
    uint8_t *layer_map = malloc(SCREEN_W * SCREEN_H);

    if (g_dither == DITHER_FS) {
        dither_fs_dual(img, pal, pal_map_a, pal_map_b, is_dual,
                       pixel_idx, layer_map);
    } else {
        /* Bayer/none: per-pixel mapping */
        for (int y = 0; y < SCREEN_H; y++)
            for (int x = 0; x < SCREEN_W; x++) {
                int i = y * SCREEN_W + x;
                int tx = x / TILE_W, ty = y / TILE_H;
                int tidx = ty * TILES_X + tx;
                int pa = pal_map_a[tidx];
                gen_color_t pixel = img[i];
                if (g_dither == DITHER_BAYER)
                    pixel = dither_pixel(pixel, x, y);
                if (is_dual[tidx]) {
                    int layer; uint8_t ci;
                    nearest_in_dual(pixel, pal[pa], pal[pal_map_b[tidx]],
                                    &layer, &ci);
                    pixel_idx[i] = ci;
                    layer_map[i] = (uint8_t)layer;
                } else {
                    pixel_idx[i] = nearest_in_pal(pixel, pal[pa],
                                                   COLORS_PER_PAL);
                    layer_map[i] = 0;
                }
            }
    }

    /* Pack BG_A tiles */
    uint8_t *out_a = tiles_a;
    for (int ty = 0; ty < TILES_Y; ty++)
        for (int tx = 0; tx < TILES_X; tx++)
            for (int py = 0; py < TILE_H; py++) {
                int sy = ty * TILE_H + py;
                for (int px = 0; px < TILE_W; px += 2) {
                    int sx = tx * TILE_W + px;
                    int i0 = sy * SCREEN_W + sx;
                    int i1 = i0 + 1;
                    uint8_t hi = (layer_map[i0] == 0) ? pixel_idx[i0] : 0;
                    uint8_t lo = (layer_map[i1] == 0) ? pixel_idx[i1] : 0;
                    *out_a++ = (hi << 4) | lo;
                }
            }

    /* Pack BG_B tiles (only dual-layer tiles, sequential) */
    uint8_t *out_b = tiles_b;
    for (int d = 0; d < dual_count; d++) {
        int tidx = dual_list[d].tile_idx;
        int tx = tidx % TILES_X, ty = tidx / TILES_X;
        for (int py = 0; py < TILE_H; py++) {
            int sy = ty * TILE_H + py;
            for (int px = 0; px < TILE_W; px += 2) {
                int sx = tx * TILE_W + px;
                int i0 = sy * SCREEN_W + sx;
                int i1 = i0 + 1;
                uint8_t hi = (layer_map[i0] == 1) ? pixel_idx[i0] : 0;
                uint8_t lo = (layer_map[i1] == 1) ? pixel_idx[i1] : 0;
                *out_b++ = (hi << 4) | lo;
            }
        }
    }

    free(pixel_idx);
    free(layer_map);
}

/* ══════════════════════════════════════════════════════════════════
 *  RLE compression (same as mandelbrot.c)
 * ══════════════════════════════════════════════════════════════════ */

static uint8_t find_token(const uint8_t *data, size_t len)
{
    uint32_t freq[256] = {0};
    for (size_t i = 0; i < len; i++) freq[data[i]]++;
    uint8_t best = 0;
    uint32_t best_count = freq[0];
    for (int i = 1; i < 256; i++) {
        if (freq[i] < best_count) {
            best_count = freq[i]; best = (uint8_t)i;
            if (best_count == 0) break;
        }
    }
    return best;
}

static size_t rle_encode(const uint8_t *in, size_t len,
                          uint8_t *out, uint8_t token)
{
    size_t oi = 0, i = 0;
    while (i < len) {
        size_t run = 1;
        while (i + run < len && in[i + run] == in[i] && run < 255)
            run++;
        if (run >= 3 || in[i] == token) {
            out[oi++] = token;
            out[oi++] = (uint8_t)run;
            out[oi++] = in[i];
            i += run;
        } else {
            for (size_t j = 0; j < run; j++)
                out[oi++] = in[i++];
        }
    }
    return oi;
}

/* ══════════════════════════════════════════════════════════════════
 *  Preview PNG writer (via stb_image_write)
 * ══════════════════════════════════════════════════════════════════ */

static int write_preview(const char *path, gen_color_t *img,
                          gen_color_t pal[4][16], uint8_t *pal_map)
{
    /* For FS mode, pre-compute all pixel indices with error diffusion */
    uint8_t *fs_idx = NULL;
    if (g_dither == DITHER_FS) {
        fs_idx = malloc(SCREEN_W * SCREEN_H);
        dither_floyd_steinberg(img, pal, pal_map, fs_idx);
    }

    uint8_t *rgb = malloc(SCREEN_W * SCREEN_H * 3);

    for (int y = 0; y < SCREEN_H; y++) {
        for (int x = 0; x < SCREEN_W; x++) {
            int tx = x / TILE_W, ty = y / TILE_H;
            int tile_idx = ty * TILES_X + tx;
            int p = pal_map[tile_idx];
            uint8_t ci;

            if (g_dither == DITHER_FS)
                ci = fs_idx[y * SCREEN_W + x];
            else
                ci = map_pixel(img[y * SCREEN_W + x], pal[p], x, y);

            /* Simulate Genesis: index 0 = backdrop (black) */
            gen_color_t mapped;
            if (ci == 0) { mapped.r = 0; mapped.g = 0; mapped.b = 0; }
            else mapped = pal[p][ci];

            int off = (y * SCREEN_W + x) * 3;
            rgb[off+0] = from_gen(mapped.r);
            rgb[off+1] = from_gen(mapped.g);
            rgb[off+2] = from_gen(mapped.b);
        }
    }

    free(fs_idx);
    int ok = stbi_write_png(path, SCREEN_W, SCREEN_H, 3, rgb, SCREEN_W * 3);
    free(rgb);

    if (!ok) { fprintf(stderr, "Failed to write %s\n", path); return -1; }
    return 0;
}

/* ══════════════════════════════════════════════════════════════════
 *  TCP push (same protocol as mandelbrot.c, extended with palette)
 * ══════════════════════════════════════════════════════════════════ */

static int send_all(int fd, const void *buf, size_t len)
{
    const uint8_t *p = buf;
    while (len > 0) {
        ssize_t n = send(fd, p, len, 0);
        if (n <= 0) { perror("send"); return -1; }
        p += n; len -= (size_t)n;
    }
    return 0;
}

static void build_frame_hdr(uint8_t *hdr, uint8_t token,
                              size_t rle_len, uint8_t flags)
{
    hdr[0] = (FRAME_MAGIC >> 8) & 0xFF;
    hdr[1] = FRAME_MAGIC & 0xFF;
    hdr[2] = token;
    hdr[3] = flags;
    hdr[4] = (rle_len >> 8) & 0xFF;
    hdr[5] = rle_len & 0xFF;
    hdr[6] = (NUM_TILES >> 8) & 0xFF;
    hdr[7] = NUM_TILES & 0xFF;
}

/* Genesis VDP color format: 0000BBB0GGG0RRR0 */
static uint16_t gen_to_vdp(gen_color_t c)
{
    return ((uint16_t)c.b << 9) | ((uint16_t)c.g << 5) | ((uint16_t)c.r << 1);
}

static int send_frame_tcp(const char *host, int port,
                           gen_color_t pal[4][16], uint8_t *pal_map,
                           const uint8_t *rle_data, size_t rle_len,
                           uint8_t token)
{
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) { perror("socket"); return -1; }

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(port);
    if (inet_pton(AF_INET, host, &addr.sin_addr) != 1) {
        fprintf(stderr, "Bad address: %s\n", host);
        close(fd); return -1;
    }

    int64_t t_connect = usec_now();
    fprintf(stderr, "Connecting to %s:%d...\n", host, port);
    if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("connect"); close(fd); return -1;
    }
    t_connect = usec_now() - t_connect;

    /* Wait for ready signal */
    int64_t t_ready = usec_now();
    {
        uint8_t ready;
        ssize_t n = recv(fd, &ready, 1, 0);
        if (n != 1) {
            fprintf(stderr, "No ready signal\n");
            close(fd); return -1;
        }
    }
    t_ready = usec_now() - t_ready;
    fprintf(stderr, "Genesis ready.\n");

    /* Build frame header — flags=0x01 means "has palette + pal_map" */
    uint8_t hdr[8];
    build_frame_hdr(hdr, token, rle_len, 0x01);

    /* Build palette block: 4 palettes × 16 colors × 2 bytes = 128 bytes */
    uint8_t pal_block[128];
    for (int p = 0; p < 4; p++) {
        for (int c = 0; c < 16; c++) {
            uint16_t vdp = gen_to_vdp(pal[p][c]);
            pal_block[(p*16+c)*2+0] = (vdp >> 8) & 0xFF;
            pal_block[(p*16+c)*2+1] = vdp & 0xFF;
        }
    }

    int64_t t_send = usec_now();
    if (send_all(fd, hdr, 8) != 0 ||
        send_all(fd, pal_block, 128) != 0 ||
        send_all(fd, pal_map, NUM_TILES) != 0 ||
        send_all(fd, rle_data, rle_len) != 0) {
        close(fd); return -1;
    }
    t_send = usec_now() - t_send;

    size_t total_sent = 8 + 128 + NUM_TILES + rle_len;

    /* Wait for ACK */
    int64_t t_ack = usec_now();
    {
        uint8_t ack;
        recv(fd, &ack, 1, 0);
    }
    t_ack = usec_now() - t_ack;

    fprintf(stderr, "\n--- TCP Timing ---\n");
    fprintf(stderr, "Connect:    %8.3f ms\n", t_connect / 1000.0);
    fprintf(stderr, "Handshake:  %8.3f ms\n", t_ready / 1000.0);
    fprintf(stderr, "Send:       %8.3f ms  (%zu bytes)\n",
            t_send / 1000.0, total_sent);
    fprintf(stderr, "ACK wait:   %8.3f ms\n", t_ack / 1000.0);
    fprintf(stderr, "Total:      %8.3f ms\n",
            (t_connect + t_ready + t_send + t_ack) / 1000.0);

    close(fd);
    return 0;
}

/*
 * Send dual-layer frame over TCP.
 * Protocol:
 *   HDR (8 bytes, flags=0x03: has_palette + dual_layer)
 *   palette (128 bytes)
 *   pal_map_a (1120 bytes)
 *   dual_count (2 bytes, big-endian)
 *   dual_positions (dual_count * 2 bytes, big-endian tile indices)
 *   pal_map_b_data (dual_count bytes, palette index for each BG_B tile)
 *   token_b (1 byte)
 *   rle_b_len (2 bytes, big-endian)
 *   RLE A payload
 *   RLE B payload
 */
static int send_frame_tcp_dual(const char *host, int port,
                                gen_color_t pal[4][16],
                                uint8_t *pmap_a, uint8_t *pmap_b,
                                const uint8_t *is_dual,
                                const dual_info_t *dual_list, int dual_count,
                                const uint8_t *rle_a, size_t rle_a_len,
                                uint8_t token_a,
                                const uint8_t *rle_b, size_t rle_b_len,
                                uint8_t token_b)
{
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) { perror("socket"); return -1; }

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    if (inet_pton(AF_INET, host, &addr.sin_addr) != 1) {
        fprintf(stderr, "Bad address: %s\n", host);
        close(fd); return -1;
    }

    int64_t t_connect = usec_now();
    fprintf(stderr, "Connecting to %s:%d...\n", host, port);
    if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("connect"); close(fd); return -1;
    }
    t_connect = usec_now() - t_connect;

    int64_t t_ready = usec_now();
    { uint8_t ready; if (recv(fd, &ready, 1, 0) != 1) {
        fprintf(stderr, "No ready signal\n"); close(fd); return -1; } }
    t_ready = usec_now() - t_ready;
    fprintf(stderr, "Genesis ready.\n");

    /* Header: flags=0x03 (palette + dual_layer) */
    uint8_t hdr[8];
    build_frame_hdr(hdr, token_a, rle_a_len, 0x03);

    /* Palette block */
    uint8_t pal_block[128];
    for (int p = 0; p < 4; p++)
        for (int c = 0; c < 16; c++) {
            uint16_t vdp = gen_to_vdp(pal[p][c]);
            pal_block[(p*16+c)*2+0] = (vdp >> 8) & 0xFF;
            pal_block[(p*16+c)*2+1] = vdp & 0xFF;
        }

    /* Build dual-layer metadata */
    uint8_t dc_buf[2] = { (dual_count >> 8) & 0xFF, dual_count & 0xFF };

    /* Dual positions (big-endian 16-bit tile indices) + pal_map_b */
    uint8_t *dual_meta = malloc(dual_count * 3);  /* 2 bytes pos + 1 byte pal */
    for (int d = 0; d < dual_count; d++) {
        uint16_t pos = dual_list[d].tile_idx;
        dual_meta[d * 3 + 0] = (pos >> 8) & 0xFF;
        dual_meta[d * 3 + 1] = pos & 0xFF;
        dual_meta[d * 3 + 2] = pmap_b[pos];
    }

    /* Token B + RLE B length */
    uint8_t b_hdr[3] = { token_b,
                          (rle_b_len >> 8) & 0xFF, rle_b_len & 0xFF };

    int64_t t_send = usec_now();
    size_t total_sent = 8 + 128 + NUM_TILES + 2 + dual_count * 3 + 3 +
                        rle_a_len + rle_b_len;
    if (send_all(fd, hdr, 8) != 0 ||
        send_all(fd, pal_block, 128) != 0 ||
        send_all(fd, pmap_a, NUM_TILES) != 0 ||
        send_all(fd, dc_buf, 2) != 0 ||
        send_all(fd, dual_meta, dual_count * 3) != 0 ||
        send_all(fd, b_hdr, 3) != 0 ||
        send_all(fd, rle_a, rle_a_len) != 0 ||
        send_all(fd, rle_b, rle_b_len) != 0) {
        free(dual_meta); close(fd); return -1;
    }
    t_send = usec_now() - t_send;
    free(dual_meta);

    int64_t t_ack = usec_now();
    { uint8_t ack; recv(fd, &ack, 1, 0); }
    t_ack = usec_now() - t_ack;

    fprintf(stderr, "\n--- TCP Timing (dual-layer) ---\n");
    fprintf(stderr, "Connect:    %8.3f ms\n", t_connect / 1000.0);
    fprintf(stderr, "Handshake:  %8.3f ms\n", t_ready / 1000.0);
    fprintf(stderr, "Send:       %8.3f ms  (%zu bytes)\n",
            t_send / 1000.0, total_sent);
    fprintf(stderr, "ACK wait:   %8.3f ms\n", t_ack / 1000.0);
    fprintf(stderr, "Total:      %8.3f ms\n",
            (t_connect + t_ready + t_send + t_ack) / 1000.0);

    close(fd);
    return 0;
}

/* ══════════════════════════════════════════════════════════════════
 *  Main
 * ══════════════════════════════════════════════════════════════════ */

/* ══════════════════════════════════════════════════════════════════
 *  Single-image processing pipeline
 *
 *  Returns 0 on success.  Caller owns no memory — everything is
 *  allocated and freed internally.
 * ══════════════════════════════════════════════════════════════════ */

static int process_image(const char *input, const char *out_png,
                          const char *out_bin, const char *tcp_host,
                          int tcp_port)
{
    int64_t t0, t1;

    /* ── Load image ── */
    int src_w, src_h;
    uint8_t *src_rgb = load_image(input, &src_w, &src_h);
    if (!src_rgb) return -1;
    fprintf(stderr, "Loaded: %s (%dx%d)\n", input, src_w, src_h);

    /* ── Resize to 320×224 preserving aspect ratio ── */
    t0 = usec_now();

    /* Fit source into SCREEN_W × SCREEN_H, letterbox/pillarbox as needed */
    double scale_w = (double)SCREEN_W / src_w;
    double scale_h = (double)SCREEN_H / src_h;
    double scale   = (scale_w < scale_h) ? scale_w : scale_h;
    int fit_w = (int)(src_w * scale + 0.5);
    int fit_h = (int)(src_h * scale + 0.5);
    if (fit_w > SCREEN_W) fit_w = SCREEN_W;
    if (fit_h > SCREEN_H) fit_h = SCREEN_H;

    int off_x = (SCREEN_W - fit_w) / 2;
    int off_y = (SCREEN_H - fit_h) / 2;

    uint8_t *fitted = resize_rgb(src_rgb, src_w, src_h, fit_w, fit_h);
    stbi_image_free(src_rgb);

    /* Place on black canvas */
    uint8_t *resized = calloc(SCREEN_W * SCREEN_H * 3, 1);
    for (int y = 0; y < fit_h; y++)
        memcpy(&resized[((off_y + y) * SCREEN_W + off_x) * 3],
               &fitted[y * fit_w * 3],
               fit_w * 3);
    free(fitted);

    t1 = usec_now();
    if (fit_w == SCREEN_W && fit_h == SCREEN_H)
        fprintf(stderr, "Resize: %dx%d → %dx%d  (%.3f ms)\n",
                src_w, src_h, SCREEN_W, SCREEN_H, (t1-t0)/1000.0);
    else
        fprintf(stderr, "Resize: %dx%d → %dx%d (centered %dx%d, %s)  (%.3f ms)\n",
                src_w, src_h, SCREEN_W, SCREEN_H, fit_w, fit_h,
                (off_x > 0) ? "pillarbox" : "letterbox", (t1-t0)/1000.0);

    /* ── Quantize to Genesis 9-bit color space ── */
    t0 = usec_now();
    int n_pixels = SCREEN_W * SCREEN_H;
    gen_color_t *gen_img = malloc(sizeof(gen_color_t) * n_pixels);
    for (int i = 0; i < n_pixels; i++) {
        gen_img[i].r = to_gen(resized[i*3+0]);
        gen_img[i].g = to_gen(resized[i*3+1]);
        gen_img[i].b = to_gen(resized[i*3+2]);
    }
    free(resized);
    t1 = usec_now();

    /* Count unique 9-bit colors */
    int n_unique;
    {
        uint8_t seen[512] = {0};
        n_unique = 0;
        for (int i = 0; i < n_pixels; i++) {
            uint16_t k = gen_key(gen_img[i]);
            if (!seen[k]) { seen[k] = 1; n_unique++; }
        }
        fprintf(stderr, "9-bit quantize: %d unique colors  (%.3f ms)\n",
                n_unique, (t1-t0)/1000.0);
    }

    /* ── Build palettes ── */
    gen_color_t pal[4][16];
    uint8_t *pal_map = malloc(NUM_TILES);

    if (n_unique <= COLORS_PER_PAL - 1) {
        /* ≤15 unique colors: all fit in every palette — direct 1:1 map */
        t0 = usec_now();
        gen_color_t unique_colors[COLORS_PER_PAL];
        uint8_t seen[512] = {0};
        int nc = 0;
        for (int i = 0; i < n_pixels && nc < COLORS_PER_PAL - 1; i++) {
            uint16_t k = gen_key(gen_img[i]);
            if (!seen[k]) { seen[k] = 1; unique_colors[nc++] = gen_img[i]; }
        }
        for (int p = 0; p < 4; p++) {
            pal[p][0] = (gen_color_t){0, 0, 0};
            for (int c = 0; c < nc; c++)
                pal[p][c + 1] = unique_colors[c];
            for (int c = nc + 1; c < COLORS_PER_PAL; c++)
                pal[p][c] = (gen_color_t){0, 0, 0};
        }
        /* All tiles use PAL0 (identical palettes) */
        memset(pal_map, 0, NUM_TILES);
        t1 = usec_now();
        fprintf(stderr, "Direct 1:1 palette (%d colors in all 4 palettes)  (%.3f ms)\n",
                nc, (t1-t0)/1000.0);

    } else if (n_unique <= USABLE_COLORS) {
        /* 16–60 unique colors: all fit across 4 palettes — cluster then
         * fill unused slots to maximize per-palette coverage */
        t0 = usec_now();
        gen_color_t palette60[USABLE_COLORS];
        int n_selected = select_top_colors(gen_img, n_pixels, palette60,
                                            USABLE_COLORS);
        refine_palettes(gen_img, palette60, n_selected, pal, pal_map, 5);

        /* Collect all unique colors */
        gen_color_t all_unique[USABLE_COLORS];
        uint8_t seen[512] = {0};
        int n_all = 0;
        for (int i = 0; i < n_pixels && n_all < USABLE_COLORS; i++) {
            uint16_t k = gen_key(gen_img[i]);
            if (!seen[k]) { seen[k] = 1; all_unique[n_all++] = gen_img[i]; }
        }

        /* Fill empty/black slots in each palette with colors not yet
         * present in that palette — maximizes per-tile color access */
        for (int p = 0; p < 4; p++) {
            uint8_t in_pal[512] = {0};
            for (int c = 1; c < COLORS_PER_PAL; c++)
                in_pal[gen_key(pal[p][c])] = 1;

            for (int c = 1; c < COLORS_PER_PAL; c++) {
                if (gen_key(pal[p][c]) != 0) continue; /* occupied */
                /* Find a color not yet in this palette */
                for (int u = 0; u < n_all; u++) {
                    uint16_t k = gen_key(all_unique[u]);
                    if (k == 0 || in_pal[k]) continue;
                    pal[p][c] = all_unique[u];
                    in_pal[k] = 1;
                    break;
                }
            }
        }
        assign_tiles(gen_img, pal, pal_map);
        t1 = usec_now();
        fprintf(stderr, "Palette refinement + fill (%d unique, all preserved)  (%.3f ms)\n",
                n_all, (t1-t0)/1000.0);

    } else {
        /* >60 unique colors: normal reduction pipeline */
        t0 = usec_now();
        gen_color_t palette60[USABLE_COLORS];
        int n_selected = select_top_colors(gen_img, n_pixels, palette60,
                                            USABLE_COLORS);
        t1 = usec_now();
        fprintf(stderr, "Selected %d colors by frequency  (%.3f ms)\n",
                n_selected, (t1-t0)/1000.0);

        t0 = usec_now();
        refine_palettes(gen_img, palette60, n_selected, pal, pal_map, 5);
        t1 = usec_now();
        fprintf(stderr, "Palette refinement (5 iters): %.3f ms\n",
                (t1-t0)/1000.0);
    }

    fprintf(stderr, "\nPalettes:\n");
    for (int p = 0; p < 4; p++) {
        fprintf(stderr, "  PAL%d:", p);
        for (int c = 0; c < 16; c++)
            fprintf(stderr, " %d%d%d", pal[p][c].r, pal[p][c].g, pal[p][c].b);
        fprintf(stderr, "\n");
    }

    /* ── Dual-layer analysis ── */
    dual_info_t dual_list[MAX_DUAL_TILES];
    int dual_count = 0;
    uint8_t is_dual[NUM_TILES] = {0};
    uint8_t pal_map_b[NUM_TILES] = {0};

    if (g_dual_layer && n_unique > USABLE_COLORS) {
        t0 = usec_now();
        dual_count = compute_dual_info(gen_img, pal, pal_map,
                                        dual_list, MAX_DUAL_TILES);
        for (int d = 0; d < dual_count; d++) {
            int idx = dual_list[d].tile_idx;
            is_dual[idx] = 1;
            /* BG_A gets the palette that was already assigned */
            /* BG_B gets the other palette from the pair */
            if (dual_list[d].pal_a == pal_map[idx])
                pal_map_b[idx] = dual_list[d].pal_b;
            else {
                pal_map_b[idx] = pal_map[idx];
                pal_map[idx] = dual_list[d].pal_a;
            }
        }
        t1 = usec_now();
        fprintf(stderr, "Dual-layer: %d of %d tiles benefit (%d selected)  "
                "(%.3f ms)\n", dual_count, NUM_TILES, dual_count,
                (t1-t0)/1000.0);
    }

    /* ── Process tiles: pixel mapping with dithering ── */
    t0 = usec_now();
    uint8_t *tiles = malloc(RAW_SIZE);
    uint8_t *tiles_b = NULL;
    size_t tiles_b_size = 0;

    if (dual_count > 0) {
        tiles_b_size = (size_t)dual_count * TILE_BYTES;
        tiles_b = malloc(tiles_b_size);
        process_tiles_dual(gen_img, pal, tiles, tiles_b,
                           pal_map, pal_map_b, is_dual,
                           dual_list, dual_count);
    } else {
        process_tiles(gen_img, pal, tiles, pal_map);
    }
    t1 = usec_now();
    fprintf(stderr, "\nTile processing: %.3f ms\n", (t1-t0)/1000.0);

    /* Palette usage stats */
    {
        int counts[4] = {0};
        for (int i = 0; i < NUM_TILES; i++) counts[pal_map[i]]++;
        fprintf(stderr, "Palette usage: PAL0=%d PAL1=%d PAL2=%d PAL3=%d\n",
                counts[0], counts[1], counts[2], counts[3]);
    }

    /* ── RLE compress ── */
    t0 = usec_now();
    uint8_t token = find_token(tiles, RAW_SIZE);
    uint8_t *rle_data = malloc(RAW_SIZE * 2);
    size_t rle_len = rle_encode(tiles, RAW_SIZE, rle_data, token);

    uint8_t token_b = 0;
    uint8_t *rle_data_b = NULL;
    size_t rle_len_b = 0;
    if (tiles_b) {
        token_b = find_token(tiles_b, tiles_b_size);
        rle_data_b = malloc(tiles_b_size * 2);
        rle_len_b = rle_encode(tiles_b, tiles_b_size, rle_data_b, token_b);
    }
    t1 = usec_now();
    fprintf(stderr, "RLE A: %zu bytes  (%.1f:1, token=0x%02X)\n",
            rle_len, (double)RAW_SIZE / rle_len, token);
    if (tiles_b)
        fprintf(stderr, "RLE B: %zu bytes  (%.1f:1, token=0x%02X)\n",
                rle_len_b, (double)tiles_b_size / rle_len_b, token_b);
    fprintf(stderr, "(%.3f ms)\n", (t1-t0)/1000.0);

    /* ── Output ── */
    if (out_png) {
        /* TODO: dual-layer preview compositing */
        write_preview(out_png, gen_img, pal, pal_map);
        fprintf(stderr, "Preview: %s\n", out_png);
    }

    if (out_bin) {
        /* Binary output — single layer only for now */
        FILE *fp = fopen(out_bin, "wb");
        if (fp) {
            uint8_t hdr[8];
            build_frame_hdr(hdr, token, rle_len, 0x01);
            uint8_t pal_block[128];
            for (int p = 0; p < 4; p++)
                for (int c = 0; c < 16; c++) {
                    uint16_t vdp = gen_to_vdp(pal[p][c]);
                    pal_block[(p*16+c)*2+0] = (vdp >> 8) & 0xFF;
                    pal_block[(p*16+c)*2+1] = vdp & 0xFF;
                }
            fwrite(hdr, 8, 1, fp);
            fwrite(pal_block, 128, 1, fp);
            fwrite(pal_map, NUM_TILES, 1, fp);
            fwrite(rle_data, 1, rle_len, fp);
            fclose(fp);
            fprintf(stderr, "Binary: %s (%zu bytes total)\n", out_bin,
                    8 + 128 + NUM_TILES + rle_len);
        }
    }

    if (tcp_host) {
        if (dual_count > 0)
            send_frame_tcp_dual(tcp_host, tcp_port, pal,
                                pal_map, pal_map_b, is_dual,
                                dual_list, dual_count,
                                rle_data, rle_len, token,
                                rle_data_b, rle_len_b, token_b);
        else
            send_frame_tcp(tcp_host, tcp_port, pal, pal_map,
                            rle_data, rle_len, token);
    }

    free(gen_img);
    free(tiles);
    free(tiles_b);
    free(pal_map);
    free(rle_data);
    free(rle_data_b);
    return 0;
}

/* ══════════════════════════════════════════════════════════════════
 *  Slideshow: scan directory for images, push in a loop
 * ══════════════════════════════════════════════════════════════════ */

static int is_image_ext(const char *name)
{
    const char *dot = strrchr(name, '.');
    if (!dot) return 0;
    dot++;
    const char *exts[] = {
        "png", "jpg", "jpeg", "bmp", "tga", "gif", "psd", "hdr", "pic",
        "PNG", "JPG", "JPEG", "BMP", "TGA", "GIF", "PSD", "HDR", "PIC",
        NULL
    };
    for (int i = 0; exts[i]; i++)
        if (strcmp(dot, exts[i]) == 0) return 1;
    return 0;
}

static int name_cmp(const void *a, const void *b)
{
    return strcmp(*(const char **)a, *(const char **)b);
}

static int run_slideshow(const char *dir, const char *tcp_host,
                          int tcp_port, int delay_sec)
{
    DIR *dp = opendir(dir);
    if (!dp) { perror(dir); return 1; }

    /* Collect image filenames */
    char **files = NULL;
    int n_files = 0, cap = 0;
    struct dirent *ent;
    while ((ent = readdir(dp)) != NULL) {
        if (!is_image_ext(ent->d_name)) continue;
        if (n_files >= cap) {
            cap = cap ? cap * 2 : 32;
            files = realloc(files, cap * sizeof(char *));
        }
        /* Build full path */
        size_t dlen = strlen(dir), nlen = strlen(ent->d_name);
        char *path = malloc(dlen + 1 + nlen + 1);
        sprintf(path, "%s/%s", dir, ent->d_name);
        files[n_files++] = path;
    }
    closedir(dp);

    if (n_files == 0) {
        fprintf(stderr, "No images found in %s\n", dir);
        free(files);
        return 1;
    }

    /* Sort alphabetically for predictable order */
    qsort(files, n_files, sizeof(char *), name_cmp);

    fprintf(stderr, "Slideshow: %d images in %s  (Ctrl-C to stop)\n",
            n_files, dir);

    /* Install SIGINT handler */
    struct sigaction sa;
    sa.sa_handler = sigint_handler;
    sa.sa_flags = 0;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT, &sa, NULL);

    int idx = 0;
    while (!g_stop) {
        fprintf(stderr, "\n══ [%d/%d] %s ══\n", idx + 1, n_files, files[idx]);
        process_image(files[idx], NULL, NULL, tcp_host, tcp_port);

        idx = (idx + 1) % n_files;

        /* Wait delay_sec seconds, checking g_stop each second */
        if (!g_stop) {
            fprintf(stderr, "Next image in %d seconds... (Ctrl-C to stop)\n",
                    delay_sec);
            for (int s = 0; s < delay_sec && !g_stop; s++)
                sleep(1);
        }
    }

    fprintf(stderr, "\nSlideshow stopped.\n");
    for (int i = 0; i < n_files; i++) free(files[i]);
    free(files);
    return 0;
}

/* ══════════════════════════════════════════════════════════════════
 *  Main
 * ══════════════════════════════════════════════════════════════════ */

static void usage(const char *prog)
{
    fprintf(stderr,
        "Usage: %s IMAGE [-o preview.png] [-b frame.bin]\n"
        "       %s IMAGE -H host [-p port] [-d mode]\n"
        "       %s -s DIR  -H host [-p port] [-d mode] [-t secs]\n"
        "\n"
        "Supported formats: PNG, JPEG, BMP, TGA, GIF, PSD, HDR, PIC\n"
        "\n"
        "Options:\n"
        "  -o FILE   Write preview PNG (Genesis-accurate colors)\n"
        "  -b FILE   Write binary frame file\n"
        "  -H HOST   Push frame to Genesis via TCP\n"
        "  -p PORT   TCP port (default: 2026)\n"
        "  -d MODE   Dither: fs (default), bayer, none\n"
        "  -s DIR    Slideshow: loop images in DIR (Ctrl-C to stop)\n"
        "  -t SECS   Slideshow delay between frames (default: 10)\n"
        "  -2        Dual-layer mode: use BG_A + BG_B for 30 colors/tile\n",
        prog, prog, prog);
}

int main(int argc, char *argv[])
{
    const char *out_png = NULL;
    const char *out_bin = NULL;
    const char *tcp_host = NULL;
    const char *slide_dir = NULL;
    int tcp_port = 2026;
    int slide_delay = 10;

    int opt;
    while ((opt = getopt(argc, argv, "o:b:H:p:d:s:t:2h")) != -1) {
        switch (opt) {
        case 'o': out_png = optarg; break;
        case 'b': out_bin = optarg; break;
        case 'H': tcp_host = optarg; break;
        case 'p': tcp_port = atoi(optarg); break;
        case 's': slide_dir = optarg; break;
        case 't': slide_delay = atoi(optarg); break;
        case 'd':
            if (strcmp(optarg, "bayer") == 0) g_dither = DITHER_BAYER;
            else if (strcmp(optarg, "none") == 0) g_dither = DITHER_NONE;
            else if (strcmp(optarg, "fs") == 0) g_dither = DITHER_FS;
            else { fprintf(stderr, "Unknown dither: %s\n", optarg);
                   usage(argv[0]); return 1; }
            break;
        case '2': g_dual_layer = 1; break;
        case 'h': usage(argv[0]); return 0;
        default:  usage(argv[0]); return 1;
        }
    }

    /* Slideshow mode */
    if (slide_dir) {
        if (!tcp_host) {
            fprintf(stderr, "Slideshow requires -H host\n");
            return 1;
        }
        return run_slideshow(slide_dir, tcp_host, tcp_port, slide_delay);
    }

    /* Single image mode */
    if (optind >= argc) {
        fprintf(stderr, "Error: no input image\n");
        usage(argv[0]);
        return 1;
    }
    const char *input = argv[optind];

    if (!out_png && !out_bin && !tcp_host)
        out_png = "preview.png";

    return process_image(input, out_png, out_bin, tcp_host, tcp_port);
}
