/*
 * image_tester.c — MegaWiFi Image Tester for Sega Genesis
 *
 * Receives photo frames over TCP from imgconv CLI tool, displays
 * full-screen 320x224 images with 4 palettes of 16 colors.
 *
 * Controls:
 *   A     — Toggle OSD debug messages
 *   Start — Toggle palette info overlay (color count + swatch grid)
 */

#include <genesis.h>
#include <task.h>
#include <string.h>
#include "ext/mw/megawifi.h"
#include "ext/mw/mw-msg.h"

extern void mw_set_draw_hook(void (*hook)(void));

#define MS_TO_FRAMES(ms)  ((((ms) * 60 / 500) + 1) / 2)
#define FPS               60

#define AP_SLOT           0
#define AP_SSID           "ATThSVWdE9"
#define AP_PASS           "bnfx6wrc#zag"
#define RECV_PORT         2026
#define RECV_CH           1
#define RECV_TIMEOUT      MS_TO_FRAMES(10000)
#define CONN_TIMEOUT      (FPS * 300)

/* Screen row assignments */
#define ROW_TITLE         0
#define ROW_FW            2
#define ROW_STATE         4
#define ROW_STATS         5
#define ROW_LOG_HDR       7
#define ROW_LOG_START     8
#define ROW_LOG_END       26

/* Tile geometry */
#define TILES_X           40
#define TILES_Y           28
#define NUM_TILES         (TILES_X * TILES_Y)     /* 1120 */
#define TILE_BYTES        32                       /* 8x8 x 4bpp */
#define RAW_SIZE          (NUM_TILES * TILE_BYTES) /* 35840 */

/* Frame header wire format (all multi-byte fields big-endian) */
#define FRAME_MAGIC       0x4D42
#define FRAME_HDR_SIZE    8

static uint16_t cmd_buf[MW_BUFLEN / 2];
static u32 tile_buf[NUM_TILES * (TILE_BYTES / 4)];  /* 35840 bytes */

static uint8_t fw_major = 0, fw_minor = 0;
static char fw_variant_buf[32] = "?";

static uint32_t g_sessions;
static uint32_t g_frames;

/* ---- OSD toggle (A button) + Palette info (Start button) ---- */

static bool osd_on = TRUE;             /* messages visible? (on during boot) */
static bool has_frame = FALSE;         /* have we displayed a frame? */
static u8 pal_info_state = 0;         /* 0=image, 1=image+pal, 2=pal on black */
static u16 prev_joy = 0;

/* Photo mode buffers */
static uint8_t photo_pal_block[128];          /* 4 x 16 x 2 VDP colors */
static uint8_t photo_pal_map[NUM_TILES];      /* palette index per tile */

/* Dual-layer (BG_B) metadata — tile data reuses cmd_buf for streaming */
#define MAX_DUAL_TILES  128
static uint16_t dual_positions[MAX_DUAL_TILES];
static uint8_t  dual_pal_map[MAX_DUAL_TILES];
static uint16_t dual_count = 0;

static void refresh_display(void);     /* forward decl */
static void show_palette_info(bool on_black);  /* forward decl */
static void restore_image(void);       /* forward decl */

static void draw_hook(void)
{
    JOY_update();
    u16 joy = JOY_readJoypad(JOY_1);
    u16 pressed = joy & ~prev_joy;
    prev_joy = joy;

    if (pressed & BUTTON_A) {
        osd_on = !osd_on;
        if (!osd_on && has_frame && pal_info_state == 0) {
            refresh_display();
        }
    }

    if ((pressed & BUTTON_START) && has_frame) {
        pal_info_state = (pal_info_state + 1) % 3;
        switch (pal_info_state) {
        case 0:  restore_image(); break;
        case 1:  show_palette_info(FALSE); break;
        case 2:  show_palette_info(TRUE); break;
        }
    }
}

/* ---- Helpers ---- */

static void clear_row(u16 row)
{
    VDP_drawText("                                        ", 0, row);
}

static void show_state(u16 pal, const char *msg)
{
    if (!osd_on && pal != PAL2) return;
    VDP_setTextPalette(pal);
    clear_row(ROW_STATE);
    VDP_drawText(msg, 1, ROW_STATE);
    VDP_setTextPalette(PAL0);
}

static void show_stats(const char *msg)
{
    if (!osd_on) return;
    clear_row(ROW_STATS);
    VDP_drawText(msg, 1, ROW_STATS);
}

static u16 log_line = ROW_LOG_START;

static void log_msg(const char *msg)
{
    if (!osd_on) return;
    if (log_line > ROW_LOG_END) {
        for (u16 r = ROW_LOG_START; r <= ROW_LOG_END; r++)
            clear_row(r);
        log_line = ROW_LOG_START;
    }
    VDP_setTextPalette(PAL3);
    clear_row(log_line);
    VDP_drawText(msg, 1, log_line);
    VDP_setTextPalette(PAL0);
    log_line++;
}

/* ---- MegaWifi init ---- */

static void user_tsk(void)
{
    while (1) mw_process();
}

static bool megawifi_init(void)
{
    char *variant = NULL;
    struct mw_ip_cfg dhcp_cfg = { {0}, {0}, {0}, {0}, {0} };
    enum mw_err err;

    if (mw_init(cmd_buf, MW_BUFLEN) != MW_ERR_NONE) return false;
    TSK_userSet(user_tsk);

    err = mw_detect(&fw_major, &fw_minor, &variant);
    if (err != MW_ERR_NONE) return false;
    if (variant) {
        uint8_t i;
        for (i = 0; i < sizeof(fw_variant_buf) - 1 && variant[i]; i++)
            fw_variant_buf[i] = variant[i];
        fw_variant_buf[i] = '\0';
    }

    err = mw_ap_cfg_set(AP_SLOT, AP_SSID, AP_PASS, MW_PHY_11BGN);
    if (err != MW_ERR_NONE) return false;

    err = mw_ip_cfg_set(AP_SLOT, &dhcp_cfg);
    if (err != MW_ERR_NONE) return false;

    err = mw_cfg_save();
    if (err != MW_ERR_NONE) return false;

    err = mw_ap_assoc(AP_SLOT);
    if (err != MW_ERR_NONE) return false;

    err = mw_ap_assoc_wait(30 * FPS);
    if (err != MW_ERR_NONE) return false;

    mw_sleep(3 * 60);

    return true;
}

/* ---- RLE streaming decoder ---- */

struct rle_dec {
    uint8_t token;
    uint8_t state;    /* 0=normal, 1=got_token, 2=got_count */
    uint8_t count;
    uint8_t *out;
    u32 out_pos;
    u32 out_max;
};

static void rle_init(struct rle_dec *d, uint8_t token, uint8_t *out, u32 max)
{
    d->token   = token;
    d->state   = 0;
    d->count   = 0;
    d->out     = out;
    d->out_pos = 0;
    d->out_max = max;
}

static void rle_feed(struct rle_dec *d, const uint8_t *in, u16 len)
{
    u16 i;
    for (i = 0; i < len && d->out_pos < d->out_max; i++) {
        uint8_t b = in[i];
        switch (d->state) {
        case 0:
            if (b == d->token) {
                d->state = 1;
            } else {
                d->out[d->out_pos++] = b;
            }
            break;
        case 1:
            d->count = b;
            d->state = 2;
            break;
        case 2: {
            uint8_t j;
            for (j = 0; j < d->count && d->out_pos < d->out_max; j++)
                d->out[d->out_pos++] = b;
            d->state = 0;
            break;
        }
        }
    }
}

/* ---- Receive exactly `len` bytes ---- */

static enum mw_err recv_exact(uint8_t *buf, u16 len)
{
    int16_t got = 0;
    while (got < (int16_t)len) {
        uint8_t ch = RECV_CH;
        int16_t chunk = (int16_t)len - got;
        enum mw_err err = mw_recv_sync(&ch, (char *)buf + got,
                                        &chunk, RECV_TIMEOUT);
        if (err != MW_ERR_NONE || chunk <= 0)
            return MW_ERR_RECV;
        got += chunk;
    }
    return MW_ERR_NONE;
}

/* ---- Photo VDP setup ---- */

static void setup_photo_display(void)
{
    u16 col, row, p;

    /* Load 4 palettes from received VDP color data */
    for (p = 0; p < 4; p++) {
        u16 pal_buf[16];
        u16 c;
        for (c = 0; c < 16; c++) {
            u16 off = (p * 16 + c) * 2;
            pal_buf[c] = ((u16)photo_pal_block[off] << 8)
                       | photo_pal_block[off + 1];
        }
        PAL_setColors(p * 16, pal_buf, 16, CPU);
    }

    /* Set BG_A tilemap with per-tile palette from pal_map */
    for (row = 0; row < TILES_Y; row++) {
        for (col = 0; col < TILES_X; col++) {
            u16 tile_idx = row * TILES_X + col;
            u16 pal = photo_pal_map[tile_idx];
            u16 attr = TILE_ATTR_FULL(pal, FALSE, FALSE, FALSE,
                                       TILE_USER_INDEX + tile_idx);
            VDP_setTileMapXY(BG_A, attr, col, row);
        }
    }

    /* Set BG_B tilemap for dual-layer tiles */
    VDP_clearPlane(BG_B, TRUE);
    VDP_waitDMACompletion();

    if (dual_count > 0) {
        u16 d, pos, dc, dr;
        u16 battr;
        for (d = 0; d < dual_count; d++) {
            pos = dual_positions[d];
            dc = pos % TILES_X;
            dr = pos / TILES_X;
            battr = TILE_ATTR_FULL(dual_pal_map[d], FALSE, FALSE, FALSE,
                                    TILE_USER_INDEX + NUM_TILES + d);
            VDP_setTileMapXY(BG_B, battr, dc, dr);
        }
    }
}

/* ---- Restore display after OSD toggle-off ---- */

static void refresh_display(void)
{
    setup_photo_display();
}

/* ---- Palette info overlay ---- */

/* Build a solid-color 8x8 tile (4bpp packed) for a given color index.
 * Each pixel in the tile = color_idx, packed as 2 pixels per byte. */
static void build_solid_tile(u32 *tile32, u8 color_idx)
{
    /* 4bpp: 2 pixels per byte, 4 bytes per row, 8 rows = 32 bytes = 8 u32 */
    u8 pair = (color_idx << 4) | color_idx;  /* two pixels */
    u32 row_val = ((u32)pair << 24) | ((u32)pair << 16) |
                  ((u32)pair << 8)  | (u32)pair;
    u16 i;
    for (i = 0; i < 8; i++)
        tile32[i] = row_val;
}

static void show_palette_info(bool on_black)
{
    char buf[42];
    u16 p, c;

    /* Always clear BG_B so dual-layer tiles don't bleed through */
    VDP_clearPlane(BG_B, TRUE);
    VDP_waitDMACompletion();

    if (on_black) {
        /* Write one black tile to the first slot, then point all
         * 1120 tilemap entries to it — 32 bytes DMA, not 35KB */
        u32 black_tile[8] = {0, 0, 0, 0, 0, 0, 0, 0};
        VDP_loadTileData((const u32 *)black_tile,
                         TILE_USER_INDEX, 1, DMA);
        VDP_waitDMACompletion();

        u16 attr = TILE_ATTR_FULL(PAL0, FALSE, FALSE, FALSE,
                                   TILE_USER_INDEX);
        u16 row, col;
        for (row = 0; row < TILES_Y; row++)
            for (col = 0; col < TILES_X; col++)
                VDP_setTileMapXY(BG_A, attr, col, row);
    } else {
        /* Overlay mode: restore image first so we layer on top.
         * Only DMA the safe BG_A region (BG_B lives at the end of tile_buf) */
        u16 bga_count = (dual_count > 0) ? (NUM_TILES - dual_count) : NUM_TILES;
        VDP_loadTileData(tile_buf, TILE_USER_INDEX, bga_count, DMA);
        VDP_waitDMACompletion();
        refresh_display();
    }

    /* Count unique non-black colors across all 4 palettes.
     * VDP format: 0000BBB0GGG0RRR0 — extract 9-bit key as B*64+G*8+R */
    u16 unique = 0;
    u16 seen[512];
    memset(seen, 0, sizeof(seen));
    for (p = 0; p < 4; p++) {
        for (c = 1; c < 16; c++) {  /* skip index 0 (backdrop) */
            u16 off = (p * 16 + c) * 2;
            u16 vdp = ((u16)photo_pal_block[off] << 8)
                    | photo_pal_block[off + 1];
            u16 b = (vdp >> 9) & 7;
            u16 g = (vdp >> 5) & 7;
            u16 r = (vdp >> 1) & 7;
            u16 key = (b << 6) | (g << 3) | r;
            if ((vdp != 0) && !seen[key]) {
                seen[key] = 1;
                unique++;
            }
        }
    }

    /*  Layout (fits 28 tile rows):
     *    row  8:  title text
     *    row  9:  index labels  0 1 2 ... F
     *    row 10-11: PAL0 swatches (2 rows tall, 2 cols wide each)
     *    row 13-14: PAL1
     *    row 16-17: PAL2
     *    row 19-20: PAL3
     */
    u16 base_row = 10;
    u16 swatch_col = 6;

    for (p = 0; p < 4; p++) {
        u16 ty_top = base_row + p * 3;
        u16 ty_bot = ty_top + 1;
        u16 row;

        /* Palette label */
        sprintf(buf, "PAL%d", (int)p);
        VDP_setTextPalette(PAL0);
        VDP_drawText(buf, 1, ty_top);

        for (c = 0; c < 16; c++) {
            u16 col_x = swatch_col + c * 2;

            /* Solid tile for color index c */
            u32 solid[8];
            build_solid_tile(solid, (u8)c);

            /* Write 2×2 block (2 wide × 2 tall) */
            for (row = ty_top; row <= ty_bot; row++) {
                u16 ti  = row * TILES_X + col_x;
                u16 ti2 = ti + 1;

                VDP_loadTileData((const u32 *)solid,
                                 TILE_USER_INDEX + ti, 1, DMA);
                VDP_waitDMACompletion();
                VDP_loadTileData((const u32 *)solid,
                                 TILE_USER_INDEX + ti2, 1, DMA);
                VDP_waitDMACompletion();

                u16 attr = TILE_ATTR_FULL(p, FALSE, FALSE, FALSE,
                                           TILE_USER_INDEX + ti);
                VDP_setTileMapXY(BG_A, attr, col_x, row);
                u16 attr2 = TILE_ATTR_FULL(p, FALSE, FALSE, FALSE,
                                            TILE_USER_INDEX + ti2);
                VDP_setTileMapXY(BG_A, attr2, col_x + 1, row);
            }
        }
    }

    /* Ensure PAL0 color 15 is white so text is always readable,
     * even when the image palette leaves PAL0 all-black */
    PAL_setColor(15, 0x0EEE);

    /* Title + index labels */
    sprintf(buf, "  %d unique colors  (4 x 16)", (int)unique);
    VDP_setTextPalette(PAL0);
    VDP_drawText(buf, 1, base_row - 2);
    VDP_drawText(" 0 1 2 3 4 5 6 7 8 9 A B C D E F", swatch_col, base_row - 1);
}

static void restore_image(void)
{
    /* In dual-layer mode, tile_buf has BG_B data at the end (last
     * dual_count tiles).  Only DMA the safe BG_A region so we don't
     * overwrite correct BG_A VRAM with BG_B pixel data. */
    u16 bga_count = (dual_count > 0) ? (NUM_TILES - dual_count) : NUM_TILES;
    VDP_loadTileData(tile_buf, TILE_USER_INDEX, bga_count, DMA);
    VDP_waitDMACompletion();

    /* Re-DMA BG_B tiles from the end of tile_buf where they live */
    if (dual_count > 0) {
        u32 *bgb_ptr = &tile_buf[(NUM_TILES - dual_count)
                                  * (TILE_BYTES / 4)];
        VDP_loadTileData(bgb_ptr, TILE_USER_INDEX + NUM_TILES,
                         dual_count, DMA);
        VDP_waitDMACompletion();
    }

    /* Restore tilemap palette assignments (both planes) */
    refresh_display();
}

/* ---- Server loop ---- */

static void server_loop(void)
{
    char dbg[40];

    for (;;) {
        enum mw_err err;

        /* Bind */
        show_state(PAL0, "Binding port 2026...");
        log_msg("tcp_bind...");
        if (mw_tcp_bind(RECV_CH, RECV_PORT) != MW_ERR_NONE) {
            show_state(PAL2, "Bind FAILED");
            log_msg("Bind failed, retry in 2s");
            for (u8 i = 0; i < 120; i++) VDP_waitVSync();
            continue;
        }
        log_msg("Bind OK, waiting for client");

        /* Wait for connection */
        show_state(PAL0, "Waiting for client...");
        if (mw_sock_conn_wait(RECV_CH, CONN_TIMEOUT) != MW_ERR_NONE) {
            log_msg("conn_wait timeout/err");
            goto disc;
        }

        g_sessions++;
        sprintf(dbg, "Client #%d connected", (int)g_sessions);
        show_state(PAL1, dbg);
        log_msg(dbg);

        /* Send "ready" signal */
        {
            uint8_t ready = 0x06;
            err = mw_send_sync(RECV_CH, (const char *)&ready, 1,
                               RECV_TIMEOUT);
            if (err != MW_ERR_NONE) {
                log_msg("Ready send failed");
                goto disc;
            }
        }

        /* Frame receive loop */
        for (;;) {
            uint8_t hdr_bytes[FRAME_HDR_SIZE];
            uint16_t magic, payload, tiles;
            uint8_t token, flags;
            struct rle_dec dec;
            u8 token_b = 0;
            u16 payload_b = 0;

            /* Receive 8-byte frame header */
            err = recv_exact(hdr_bytes, FRAME_HDR_SIZE);
            if (err != MW_ERR_NONE) {
                if (g_frames > 0)
                    log_msg("Client disconnected");
                else
                    log_msg("Header recv failed");
                goto disc;
            }

            /* Parse header */
            magic   = ((uint16_t)hdr_bytes[0] << 8) | hdr_bytes[1];
            token   = hdr_bytes[2];
            flags   = hdr_bytes[3];
            payload = ((uint16_t)hdr_bytes[4] << 8) | hdr_bytes[5];
            tiles   = ((uint16_t)hdr_bytes[6] << 8) | hdr_bytes[7];

            if (magic != FRAME_MAGIC) {
                sprintf(dbg, "Bad magic: 0x%04X", (int)magic);
                log_msg(dbg);
                goto disc;
            }

            if (tiles != NUM_TILES || payload == 0) {
                sprintf(dbg, "Bad frame: t=%d p=%d", (int)tiles, (int)payload);
                log_msg(dbg);
                goto disc;
            }

            /* Photo mode (flags=0x01): receive palette + pal_map */
            if (flags & 0x01) {
                err = recv_exact(photo_pal_block, 128);
                if (err != MW_ERR_NONE) {
                    log_msg("Palette recv failed");
                    goto disc;
                }
                err = recv_exact(photo_pal_map, NUM_TILES);
                if (err != MW_ERR_NONE) {
                    log_msg("Pal_map recv failed");
                    goto disc;
                }
            }

            /* Dual-layer mode (flags & 0x02): receive BG_B metadata */
            dual_count = 0;
            token_b = 0;
            payload_b = 0;
            if (flags & 0x02) {
                uint8_t dc_buf[2];
                u16 d;
                uint8_t b_hdr[3];

                err = recv_exact(dc_buf, 2);
                if (err != MW_ERR_NONE) {
                    log_msg("Dual count recv failed");
                    goto disc;
                }
                dual_count = ((u16)dc_buf[0] << 8) | dc_buf[1];
                if (dual_count > MAX_DUAL_TILES)
                    dual_count = MAX_DUAL_TILES;

                /* Receive positions + pal_map_b as one block
                 * (dual_count * 3 bytes: 2 pos + 1 pal per entry) */
                {
                    u16 meta_size = dual_count * 3;
                    uint8_t *meta = (uint8_t *)cmd_buf;  /* reuse cmd_buf */
                    err = recv_exact(meta, meta_size);
                    if (err != MW_ERR_NONE) {
                        log_msg("Dual meta recv failed");
                        goto disc;
                    }
                    for (d = 0; d < dual_count; d++) {
                        dual_positions[d] = ((u16)meta[d*3] << 8) | meta[d*3+1];
                        dual_pal_map[d] = meta[d*3+2];
                    }
                }

                /* Receive token_b + rle_b_len */
                err = recv_exact(b_hdr, 3);
                if (err != MW_ERR_NONE) {
                    log_msg("B header recv failed");
                    goto disc;
                }
                token_b = b_hdr[0];
                payload_b = ((u16)b_hdr[1] << 8) | b_hdr[2];

                sprintf(dbg, "Dual: %d tiles, B=%d bytes",
                        (int)dual_count, (int)payload_b);
                log_msg(dbg);
            }

            /* Stream-receive RLE payload A, decode into tile_buf */
            rle_init(&dec, token, (uint8_t *)tile_buf, RAW_SIZE);

            {
                u16 remaining = payload;
                while (remaining > 0) {
                    uint8_t ch = RECV_CH;
                    int16_t chunk = (remaining > (u16)MW_BUFLEN)
                                    ? (int16_t)MW_BUFLEN
                                    : (int16_t)remaining;
                    err = mw_recv_sync(&ch, (char *)cmd_buf,
                                       &chunk, RECV_TIMEOUT);
                    if (err != MW_ERR_NONE || chunk <= 0) {
                        sprintf(dbg, "Payload A recv err=%d", (int)err);
                        log_msg(dbg);
                        goto disc;
                    }
                    rle_feed(&dec, (uint8_t *)cmd_buf, (u16)chunk);
                    remaining -= (u16)chunk;
                }
            }

            /* DMA BG_A tile data to VRAM */
            VDP_loadTileData(tile_buf, TILE_USER_INDEX,
                             NUM_TILES, DMA);
            VDP_waitDMACompletion();

            /* Pass 2: receive BG_B tiles into the END of tile_buf
             * so BG_A data at the start is preserved for restore.
             * BG_A is already safe in VRAM after DMA above. */
            if (dual_count > 0 && payload_b > 0) {
                struct rle_dec dec_b;
                u16 b_raw = dual_count * TILE_BYTES;
                u16 remaining_b = payload_b;
                /* Place BG_B at end: offset = (NUM_TILES - dual_count) tiles */
                u32 *bgb_ptr = &tile_buf[(NUM_TILES - dual_count)
                                          * (TILE_BYTES / 4)];

                rle_init(&dec_b, token_b, (uint8_t *)bgb_ptr, b_raw);

                while (remaining_b > 0) {
                    uint8_t ch = RECV_CH;
                    int16_t chunk = (remaining_b > (u16)MW_BUFLEN)
                                    ? (int16_t)MW_BUFLEN
                                    : (int16_t)remaining_b;
                    err = mw_recv_sync(&ch, (char *)cmd_buf,
                                       &chunk, RECV_TIMEOUT);
                    if (err != MW_ERR_NONE || chunk <= 0) {
                        sprintf(dbg, "Payload B recv err=%d", (int)err);
                        log_msg(dbg);
                        goto disc;
                    }
                    rle_feed(&dec_b, (uint8_t *)cmd_buf, (u16)chunk);
                    remaining_b -= (u16)chunk;
                }

                /* DMA BG_B tiles right after BG_A tiles in VRAM */
                VDP_loadTileData(bgb_ptr,
                                 TILE_USER_INDEX + NUM_TILES,
                                 dual_count, DMA);
                VDP_waitDMACompletion();
            }

            /* Set up photo display */
            if (flags & 0x01)
                setup_photo_display();
            has_frame = TRUE;

            /* Dismiss palette info if it was showing */
            pal_info_state = 0;

            g_frames++;
            sprintf(dbg, "Frame %d (%d bytes)",
                    (int)g_frames, (int)payload);
            show_stats(dbg);

            /* ACK frame */
            {
                uint8_t ack = 0x06;
                mw_send_sync(RECV_CH, (const char *)&ack, 1,
                             RECV_TIMEOUT);
            }
        }

disc:
        log_msg("Closing socket...");
        mw_close(RECV_CH);
        {
            u16 i;
            for (i = 0; i < 180; i++) VDP_waitVSync();
        }
        log_msg("Ready to rebind");
    }
}

int main(bool hard_reset)
{
    (void)hard_reset;

    VDP_setScreenWidth320();
    VDP_setPlaneSize(64, 32, TRUE);
    VDP_setScrollingMode(HSCROLL_PLANE, VSCROLL_PLANE);
    VDP_clearPlane(BG_A, TRUE);
    VDP_clearPlane(BG_B, TRUE);

    /* PAL0: black bg + white text */
    PAL_setColor(0,  RGB24_TO_VDPCOLOR(0x000000));
    PAL_setColor(15, RGB24_TO_VDPCOLOR(0xFFFFFF));
    /* PAL1: green text */
    PAL_setColor(31, RGB24_TO_VDPCOLOR(0x00CC00));
    /* PAL2: red text */
    PAL_setColor(47, RGB24_TO_VDPCOLOR(0xFF2020));
    /* PAL3: blue log text */
    PAL_setColor(63, RGB24_TO_VDPCOLOR(0x00AAFF));
    VDP_waitVSync();

    VDP_setTextPalette(PAL0);
    VDP_drawText("[ MegaWifi Image Tester ]", 0, ROW_TITLE);
    JOY_init();

    show_state(PAL0, "Connecting to WiFi...");
    if (!megawifi_init()) {
        show_state(PAL2, "WiFi init FAILED");
        while (1) VDP_waitVSync();
    }

    /* Show IP briefly */
    {
        struct mw_ip_cfg *ip = NULL;
        if (mw_ip_current(&ip) == MW_ERR_NONE && ip) {
            uint32_t a = ip->addr.addr;
            char buf[42];
            sprintf(buf, "IP %d.%d.%d.%d:%d",
                    (int)((a >> 24) & 0xFF), (int)((a >> 16) & 0xFF),
                    (int)((a >> 8) & 0xFF), (int)(a & 0xFF), RECV_PORT);
            show_state(PAL1, buf);
        }
    }
    {
        u16 i;
        for (i = 0; i < 120; i++) VDP_waitVSync();
    }

    /* Clear screen — clean canvas for frames */
    VDP_clearPlane(BG_A, TRUE);
    VDP_waitVSync();
    osd_on = FALSE;

    /* Install draw hook for button polling during blocking recv */
    mw_set_draw_hook(draw_hook);

    server_loop();
    return 0;
}
