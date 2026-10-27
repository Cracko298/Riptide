/* SPDX-License-Identifier: MIT */
#include "riptide/image.h"
#include "miniz.h"
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>
static void be32(unsigned char *p, uint32_t v) {
    p[0] = v >> 24;
    p[1] = v >> 16;
    p[2] = v >> 8;
    p[3] = v;
}
static void le32(unsigned char *p, uint32_t v) {
    p[0] = v;
    p[1] = v >> 8;
    p[2] = v >> 16;
    p[3] = v >> 24;
}
static bool put(FILE *f, const void *p, size_t n) {
    return fwrite(p, 1, n, f) == n;
}
static bool chunk(FILE *f, const char *kind, const unsigned char *p, uint32_t n) {
    unsigned char h[8], c[4];
    be32(h, n);
    memcpy(h + 4, kind, 4);
    mz_ulong crc = mz_crc32(0, (const unsigned char *)kind, 4);
    if (n)
        crc = mz_crc32(crc, p, n);
    be32(c, (uint32_t)crc);
    return put(f, h, 8) && (!n || put(f, p, n)) && put(f, c, 4);
}
bool riptide_image_write(FILE *f, const unsigned char *rgba, unsigned w, unsigned h, size_t stride,
                    bool bottom_up, bool bmp) {
    if (!f || !rgba || !w || !h || w > 8192 || h > 8192 || stride < (size_t)w * 4 ||
        stride > SIZE_MAX / h || (size_t)w * h > 32000000)
        return false;
    bool ok = true;
    if (bmp) {
        unsigned char header[54] = {0};
        header[0] = 'B';
        header[1] = 'M';
        le32(header + 2, 54 + w * h * 4);
        le32(header + 10, 54);
        le32(header + 14, 40);
        le32(header + 18, w);
        le32(header + 22, 0u - h);
        header[26] = 1;
        header[28] = 32;
        le32(header + 34, w * h * 4);
        unsigned char *row = malloc((size_t)w * 4);
        if (!row)
            return false;
        ok = put(f, header, sizeof header);
        for (unsigned y = 0; y < h && ok; y++) {
            const unsigned char *src = rgba + (bottom_up ? h - 1 - y : y) * stride;
            for (unsigned x = 0; x < w; x++) {
                row[4 * x] = src[4 * x + 2];
                row[4 * x + 1] = src[4 * x + 1];
                row[4 * x + 2] = src[4 * x];
                row[4 * x + 3] = 255;
            }
            ok = put(f, row, (size_t)w * 4);
        }
        free(row);
    } else {
        size_t pitch = (size_t)w * 3 + 1, bytes = pitch * h;
        unsigned char *raw = malloc(bytes);
        if (!raw)
            return false;
        for (unsigned y = 0; y < h; y++) {
            const unsigned char *src = rgba + (bottom_up ? h - 1 - y : y) * stride;
            unsigned char *dst = raw + y * pitch;
            *dst++ = 0;
            for (unsigned x = 0; x < w; x++) {
                memcpy(dst, src + 4 * x, 3);
                dst += 3;
            }
        }
        mz_ulong size = mz_compressBound((mz_ulong)bytes);
        unsigned char *packed = malloc(size);
        if (!packed) {
            free(raw);
            return false;
        }
        int result = mz_compress2(packed, &size, raw, (mz_ulong)bytes, 3);
        free(raw);
        if (result != MZ_OK) {
            free(packed);
            return false;
        }
        const unsigned char signature[] = {137, 80, 78, 71, 13, 10, 26, 10};
        unsigned char ihdr[13] = {0};
        be32(ihdr, w);
        be32(ihdr + 4, h);
        ihdr[8] = 8;
        ihdr[9] = 2;
        ok = put(f, signature, 8) && chunk(f, "IHDR", ihdr, 13) &&
             chunk(f, "IDAT", packed, (uint32_t)size) && chunk(f, "IEND", NULL, 0);
        free(packed);
    }
    return ok && !ferror(f);
}
static bool image_is_bmp(const char *path) {
    if (!path)
        return false;
    size_t n = strlen(path);
    return n >= 4 && path[n - 4] == '.' && tolower((unsigned char)path[n - 3]) == 'b' &&
           tolower((unsigned char)path[n - 2]) == 'm' && tolower((unsigned char)path[n - 1]) == 'p';
}
bool riptide_image_save(const char *path, const unsigned char *p, unsigned w, unsigned h, size_t stride,
                   bool bottom_up) {
    if (!path || !p)
        return false;
    FILE *f = fopen(path, "wb");
    if (!f)
        return false;
    bool ok = riptide_image_write(f, p, w, h, stride, bottom_up, image_is_bmp(path));
    if (fclose(f))
        ok = false;
    if (!ok)
        remove(path);
    return ok;
}
