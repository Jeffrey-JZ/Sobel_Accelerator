#include <stdio.h>

#include "sobel.h"

static void print_image(const uint8_t *image, size_t width, size_t height)
{
    for (size_t y = 0; y < height; ++y) {
        for (size_t x = 0; x < width; ++x) {
            printf("%3u ", image[y * width + x]);
        }
        putchar('\n');
    }
}

int main(void)
{
    /* Simple 8x8 gradient test pattern with a bright square in the center. */
    const size_t width = 8;
    const size_t height = 8;
    const uint8_t input[] = {
        10,  10,  10,  10,  10,  10,  10,  10,
        10,  40,  40,  40,  40,  40,  40,  10,
        10,  40, 200, 200, 200, 200,  40,  10,
        10,  40, 200, 250, 250, 200,  40,  10,
        10,  40, 200, 250, 250, 200,  40,  10,
        10,  40, 200, 200, 200, 200,  40,  10,
        10,  40,  40,  40,  40,  40,  40,  10,
        10,  10,  10,  10,  10,  10,  10,  10
    };

    uint8_t output[sizeof(input)] = {0};
    const uint8_t threshold = 100;

    sobel_edge(input, output, width, height, threshold);

    printf("Input image (grayscale values):\n");
    print_image(input, width, height);

    printf("\nSobel edges (threshold = %u):\n", threshold);
    print_image(output, width, height);

    return 0;
}