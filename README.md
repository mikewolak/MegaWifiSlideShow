# MegaWifi SlideShow

Display photographs and artwork on a real Sega Genesis / Mega Drive via WiFi.

A PC-side tool converts any image (PNG, JPEG, BMP, TGA, GIF, PSD, HDR, PIC) into Genesis VDP tile data and streams it over TCP to a Genesis running the receiver ROM through a [MegaWifi](https://www.intergalaktik.eu/megawifi.html) cartridge.

## Features

- **Full-screen 320x224 display** with 4 palettes of 16 colors (60 usable + transparency)
- **Dual background layer mode** (`-2`): composites BG_A + BG_B for up to 30 colors per tile instead of 15
- **Floyd-Steinberg dithering** (default), ordered Bayer dithering, or no dithering
- **Aspect ratio preservation**: letterbox/pillarbox with black borders to maintain source proportions
- **Smart palette allocation**: k-means clustering with luminance-seeded centroids, cross-palette deduplication with frequency thresholds, dead-tile exclusion for border regions
- **Direct palette mapping** for images with 60 or fewer unique 9-bit colors (no color loss)
- **RLE compression** for efficient WiFi transfer (~1.5:1 typical)
- **Slideshow mode**: loop through a directory of images with configurable delay
- **On-screen palette inspector**: Start button toggles a color swatch overlay showing all 4 palettes and unique color count
- **OSD debug messages**: A button toggles connection/transfer status text

## Hardware Requirements

- Sega Genesis / Mega Drive console
- [MegaWifi cartridge](https://www.intergalaktik.eu/megawifi.html) (ESP32-C3 WiFi module + flash programmer)
- A PC on the same WiFi network (macOS, Linux, or any POSIX system with a C99 compiler)

## Repository Structure

```
MegaWifiSlideShow/
├── cli/                    # PC-side image converter
│   ├── Makefile
│   ├── imgconv.c           # Main converter + TCP sender
│   ├── stb_image.h         # Bundled image loader (nothings/stb)
│   ├── stb_image_write.h   # Bundled image writer (nothings/stb)
│   └── testimg/            # 10 sample images (classic test images)
│       ├── airplane.png
│       ├── barbara.png
│       ├── cameraman.png
│       ├── goldhill.png
│       ├── kingtut.png     # Amiga Deluxe Paint II classic (32 colors)
│       ├── lena.png
│       ├── mandrill.png
│       ├── peppers.png
│       ├── sailboat.png
│       └── splash.png
└── genesis/                # Genesis receiver ROM
    ├── Makefile            # SGDK-based build (requires m68k-elf-gcc + SGDK)
    └── src/
        ├── image_tester.c  # Main ROM: TCP server, RLE decoder, VDP display
        ├── megawifi.c      # MegaWifi API override (draw hook + timeout fixes)
        └── config.h        # Module config
```

## Building

### PC Tool (imgconv)

```bash
cd cli
make
```

Requires only a C99 compiler and libm. No external dependencies — image I/O uses the bundled stb headers.

### Genesis ROM

```bash
cd genesis
make
```

Requires:
- **m68k-elf-gcc** cross-compiler (GCC for Motorola 68000)
- **SGDK** (Sega Genesis Development Kit) installed at `$HOME/sgdk`
  - Override the path by editing the `GDK` variable in the Makefile

The ROM is output to `genesis/out/image_tester.bin`. Flash it to your MegaWifi cartridge using the [mdma](https://gitlab.com/doragasu/mw-mdma-cli) programmer tool.

## Usage

### Single Image

Convert and display one image on the Genesis:

```bash
# Push to Genesis at 192.168.1.199 (default port 2026)
./imgconv image.png -H 192.168.1.199

# Dual-layer mode for more colors per tile
./imgconv image.png -H 192.168.1.199 -2

# Save a preview PNG (Genesis-accurate rendering, no hardware needed)
./imgconv image.png -o preview.png

# Save binary frame file
./imgconv image.png -b frame.bin
```

### Slideshow

Loop through all images in a directory:

```bash
# Default 10-second delay
./imgconv -s testimg -H 192.168.1.199

# 5-second delay, dual-layer mode
./imgconv -s testimg -H 192.168.1.199 -t 5 -2
```

Press Ctrl-C to stop.

### Dithering Options

```bash
./imgconv image.png -H host -d fs      # Floyd-Steinberg (default)
./imgconv image.png -H host -d bayer   # Ordered Bayer 4x4
./imgconv image.png -H host -d none    # No dithering
```

### Genesis Controls

| Button | Action |
|--------|--------|
| **Start** | Cycle palette overlay: off → overlay on image → overlay on black → off |
| **A** | Toggle OSD debug text (connection status, transfer stats) |

### Full Options

```
Usage: imgconv IMAGE [-o preview.png] [-b frame.bin]
       imgconv IMAGE -H host [-p port] [-d mode]
       imgconv -s DIR  -H host [-p port] [-d mode] [-t secs]

Options:
  -o FILE   Write preview PNG (Genesis-accurate colors)
  -b FILE   Write binary frame file
  -H HOST   Push frame to Genesis via TCP
  -p PORT   TCP port (default: 2026)
  -d MODE   Dither: fs (default), bayer, none
  -s DIR    Slideshow: loop images in DIR (Ctrl-C to stop)
  -t SECS   Slideshow delay between frames (default: 10)
  -2        Dual-layer mode: use BG_A + BG_B for 30 colors/tile
```

## How It Works

### Color Pipeline

1. **Load** any image format via stb_image
2. **Resize** to fit 320x224 with aspect ratio preservation (letterbox/pillarbox)
3. **Quantize** to Genesis 9-bit color space (3 bits per channel = 512 possible colors)
4. **Select** top 60 colors by pixel frequency
5. **Cluster** tiles into 4 groups by luminance using k-means (black border tiles excluded from clustering)
6. **Build palettes**: each cluster gets its 15 most-used colors + index 0 (transparent/black)
7. **Deduplicate** across palettes with a 20% frequency threshold (shared colors kept where genuinely needed)
8. **Dither** with Floyd-Steinberg error diffusion (palette-aware, per-tile)
9. **Encode** as 4bpp tile data (8x8 pixels, 32 bytes per tile, 1120 tiles per frame)
10. **Compress** with RLE and stream over TCP

### Dual Background Layer Mode

The Genesis VDP has two background planes: BG_A (foreground) and BG_B (background). Pixel index 0 is transparent on both planes, so BG_B shows through where BG_A has transparency.

In dual-layer mode (`-2`), tiles that would benefit from two palettes are scored and the best 128 are selected. For each dual tile:
- BG_A renders pixels best served by palette X
- BG_B renders pixels best served by palette Y
- Pixels not needed on a layer are set to index 0 (transparent)

This gives up to **30 colors per tile** instead of 15, at the cost of 128 additional tiles in VRAM.

### Wire Protocol

Each frame is sent as a single TCP message:

```
Header (8 bytes):
  magic:    u16  0x4D42 ("MB")
  flags:    u8   bit 0 = has palette, bit 1 = dual layer
  token:    u8   RLE escape byte for BG_A
  rle_len:  u16  compressed payload size (BG_A)
  raw_len:  u16  decompressed size (35840 = 1120 tiles × 32 bytes)

If flags & 0x01 (palette):
  pal_block:  128 bytes  (4 palettes × 16 colors × 2 bytes, VDP format)
  pal_map:    1120 bytes (palette index per tile)

If flags & 0x02 (dual layer):
  dual_count:     u16
  dual_meta:      dual_count × 3 bytes (2 position + 1 pal_index per tile)
  token_b:        u8   RLE escape byte for BG_B
  rle_b_len:      u16  compressed BG_B payload size

RLE payload A: rle_len bytes
RLE payload B: rle_b_len bytes (if dual layer)
```

### Genesis RAM Budget

The Genesis has 64KB of work RAM. The receiver uses a two-pass approach to avoid RAM overflow:
1. Receive and RLE-decode BG_A tiles into a 35KB buffer, DMA to VRAM
2. Receive BG_B tiles into the tail end of the same buffer, DMA to a separate VRAM region

This keeps total BSS under 42KB, leaving sufficient stack space for SGDK.

## Sample Images

The `cli/testimg/` directory includes 10 classic test images:

| Image | Description | Source |
|-------|-------------|--------|
| airplane.png | F-16 fighter jet | USC-SIPI |
| barbara.png | Woman with scarf | USC-SIPI |
| cameraman.png | Man with tripod camera | MIT |
| goldhill.png | English village | USC-SIPI |
| kingtut.png | King Tutankhamun mask | Deluxe Paint II (Amiga, 32 colors) |
| lena.png | Woman in hat | USC-SIPI |
| mandrill.png | Mandrill baboon face | USC-SIPI |
| peppers.png | Bell peppers | USC-SIPI |
| sailboat.png | Sailboat on lake | USC-SIPI |
| splash.png | Woman in pool | USC-SIPI |

## License

This project uses the MegaWifi API by [doragasu](https://gitlab.com/doragasu). Image loading via [stb](https://github.com/nothings/stb) (public domain). Sample test images are from the USC-SIPI Image Database and other public domain sources.
