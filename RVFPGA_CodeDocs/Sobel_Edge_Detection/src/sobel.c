// Implementation of the Sobel algorithm
// Author: Junze Jiang
// 17/11/2025

// Include head file & abs functions
#include "sobel.h"
#include <stdlib.h>

// Sobel convolution kernel of x direction
static const int8_t SOBEL_X[3][3] = {
    { 1,  0, -1},
    { 2,  0, -2},
    { 1,  0, -1}
};

// Sobel convolution kernel of y direction
static const int8_t SOBEL_Y[3][3] = {
    { 1,  2,  1},
    { 0,  0,  0},
    {-1, -2, -1}
};

// Auxiliary function: Clamping int values to 0–255
// During Sobel calculation, the sum of gx and gy may exceed 255 or fall below 0
// This function performs clipping: Less than 0 → 0; Greater than 255 → 255
// Otherwise, directly convert to uint8_t
static uint8_t clamp_int_to_byte(int value)
{
    if (value < 0) {
        return 0;
    }
    if (value > 255) {
        return 255;
    }
    return (uint8_t)value;
}

// Core function: Achieve the Sobel Algorithm
void sobel_edge(const uint8_t *input, uint8_t *output,
                size_t width, size_t height, uint8_t threshold)
{
    // Reset the entire output to zero, so that when we do not process boundary pixels, they are 0 (black)
    for (size_t i = 0; i < width * height; ++i) {
        output[i] = 0;
    }

    // Iterate through non-boundary pixels: 1 <= x <= width-2, 1 <= y <= height-2
    // Initialise gx and gy to 0 at each position.
    for (size_t y = 1; y + 1 < height; ++y) {
        for (size_t x = 1; x + 1 < width; ++x) {
            int gx = 0;
            int gy = 0;

            // Perform convolution on the 3×3 neighbourhood of the current pixel
            // (y + ky - 1, x + kx - 1) are the neighbourhood coordinates
            // gx: multiplied by SOBEL_X
            // gy: multiplied by SOBEL_Y
            for (size_t ky = 0; ky < 3; ++ky) {
                for (size_t kx = 0; kx < 3; ++kx) {
                    uint8_t pixel = input[(y + ky - 1) * width + (x + kx - 1)];
                    gx += pixel * SOBEL_X[ky][kx];
                    gy += pixel * SOBEL_Y[ky][kx];
                }
            }

            // Use |gx| + |gy| as edge strength (gradient magnitude)
            // If magnitude > threshold: Clamp to 0–255 via clamp_int_to_byte and store in output
            // Otherwise: Set to 0 (discarding weak edges/noise)
            int magnitude = abs(gx) + abs(gy);
            output[y * width + x] = (magnitude > threshold)
                                        ? clamp_int_to_byte(magnitude)
                                        : 0;
        }
    }
}