/* SPDX-License-Identifier: MIT */
#ifndef RIPTIDE_IMAGE_H
#define RIPTIDE_IMAGE_H
#include "core.h"
#include <stdio.h>
#ifdef __cplusplus
extern "C" {
#endif
/** Encode a framebuffer into an existing stream.
 * @param file Writable binary stream; caller retains ownership and must check fclose.
 * @param rgba Source RGBA8 pixels; alpha is discarded.
 * @param width Width in [1, 8192] pixels.
 * @param height Height in [1, 8192]; total pixels must not exceed 32 million.
 * @param stride Source row size in bytes, at least width*4.
 * @param bottom_up True if the source starts with the bottom row.
 * @param bmp True selects BMP; false selects PNG.
 * @return False on validation, allocation or write failure. A failed stream may contain partial
 * data. The caller supplies at least stride*height readable bytes. Compression allocates temporary
 * storage. */
bool riptide_image_write(FILE *file, const unsigned char *rgba, unsigned width, unsigned height,
                    size_t stride, bool bottom_up, bool bmp);
/** Save a framebuffer, choosing PNG unless the extension is .bmp (case insensitive).
 * @param path Destination path; an existing file is overwritten.
 * @param rgba RGBA8 source; alpha is discarded.
 * @param width Width in pixels.
 * @param height Height in pixels.
 * @param stride Source bytes per row.
 * @param bottom_up True for a bottom-row-first source.
 * @return True only if encoding and close succeed. Removes the destination on failure after open.
 * Uses the same size limits as riptide_image_write; this is not an atomic replace operation. */
bool riptide_image_save(const char *path, const unsigned char *rgba, unsigned width, unsigned height,
                   size_t stride, bool bottom_up);
#ifdef __cplusplus
}
#endif
#endif
