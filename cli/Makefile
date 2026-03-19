# Makefile — MegaWifi SlideShow: PC-side image converter
#
# Pure C99, no external dependencies beyond libc + libm.
# stb_image + stb_image_write are bundled single-header libs.

CC       = cc
CFLAGS   = -std=c99 -O2 -Wall -Wextra

.PHONY: all clean

all: imgconv

imgconv: imgconv.c stb_image.h stb_image_write.h
	$(CC) $(CFLAGS) $< -lm -o $@

clean:
	rm -f imgconv
