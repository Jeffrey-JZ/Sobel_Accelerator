// The Sobel operator's header file
// It declares the function prototype for the sobel_edge function, enabling main.c to call it.
// Author: Junze Jiang
// 17/11/2025

#ifndef SOBEL_H
#define SOBEL_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Apply Sobel edge detection to a grayscale image.
 *
 * @param input      Pointer to a width*height grayscale buffer (0-255 per pixel).
 * @param output     Pointer to caller-allocated buffer of the same size that will
 *                   receive the edge magnitudes (0-255). Border pixels are set to 0.
 * @param width      Image width in pixels.
 * @param height     Image height in pixels.
 * @param threshold  Gradients with magnitude <= threshold are suppressed (0).
 */
void sobel_edge(const uint8_t *input, uint8_t *output,
                size_t width, size_t height, uint8_t threshold); // unit8_t/size_t are from <stdint.h> & <stddef.h>

#ifdef __cplusplus
}
#endif

#endif /* SOBEL_H */