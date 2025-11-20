// Simple PGM-based host demo for Sobel edge detection.
// Reads a binary P5 PGM grayscale image and writes another P5 PGM with edges.

#include "sobel.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int read_next_token(FILE *fp, char *buffer, size_t buf_size)
{
    int ch;
    size_t idx = 0;

    // Skip whitespace and comments.
    while ((ch = fgetc(fp)) != EOF) {
        if (ch == '#') {
            // Skip comment line.
            while ((ch = fgetc(fp)) != EOF && ch != '\n') {
                ;
            }
            continue;
        }
        if (ch > ' ') {
            break;
        }
    }

    if (ch == EOF) {
        return 0;
    }

    buffer[idx++] = (char)ch;
    while (idx + 1 < buf_size && (ch = fgetc(fp)) != EOF && ch > ' ') {
        buffer[idx++] = (char)ch;
    }
    buffer[idx] = '\0';
    return 1;
}

static uint8_t *load_pgm(const char *path, size_t *width, size_t *height)
{
    FILE *fp = fopen(path, "rb");
    if (!fp) {
        fprintf(stderr, "Failed to open %s\n", path);
        return NULL;
    }

    char token[64];
    if (!read_next_token(fp, token, sizeof(token)) || strcmp(token, "P5") != 0) {
        fprintf(stderr, "Only binary P5 PGM is supported.\n");
        fclose(fp);
        return NULL;
    }

    if (!read_next_token(fp, token, sizeof(token))) {
        fclose(fp);
        return NULL;
    }
    *width = (size_t)strtoul(token, NULL, 10);

    if (!read_next_token(fp, token, sizeof(token))) {
        fclose(fp);
        return NULL;
    }
    *height = (size_t)strtoul(token, NULL, 10);

    if (!read_next_token(fp, token, sizeof(token))) {
        fclose(fp);
        return NULL;
    }
    int maxval = atoi(token);
    if (maxval != 255) {
        fprintf(stderr, "Expected maxval 255, got %d\n", maxval);
        fclose(fp);
        return NULL;
    }

    // Skip a single whitespace sequence after header, then rewind one byte
    // so the first pixel stays in the stream.
    int c;
    do {
        c = fgetc(fp);
    } while (c == ' ' || c == '\n' || c == '\r' || c == '\t');
    if (c != EOF) {
        ungetc(c, fp);
    }

    size_t pixels = (*width) * (*height);
    uint8_t *data = (uint8_t *)malloc(pixels);
    if (!data) {
        fclose(fp);
        return NULL;
    }

    size_t read = fread(data, 1, pixels, fp);
    fclose(fp);
    if (read != pixels) {
        fprintf(stderr, "Unexpected EOF while reading pixels\n");
        free(data);
        return NULL;
    }

    return data;
}

static int write_pgm(const char *path, const uint8_t *data, size_t width, size_t height)
{
    FILE *fp = fopen(path, "wb");
    if (!fp) {
        return 0;
    }
    fprintf(fp, "P5\n%zu %zu\n255\n", width, height);
    fwrite(data, 1, width * height, fp);
    fclose(fp);
    return 1;
}

int main(int argc, char **argv)
{
    if (argc < 3 || argc > 4) {
        fprintf(stderr, "Usage: %s <input.pgm> <output.pgm> [threshold]\n", argv[0]);
        return 1;
    }

    const char *input_path = argv[1];
    const char *output_path = argv[2];
    uint8_t threshold = (argc == 4) ? (uint8_t)atoi(argv[3]) : 80;

    size_t width = 0, height = 0;
    uint8_t *input = load_pgm(input_path, &width, &height);
    if (!input) {
        return 1;
    }

    uint8_t *edges = (uint8_t *)calloc(width * height, 1);
    if (!edges) {
        free(input);
        return 1;
    }

    sobel_edge(input, edges, width, height, threshold);

    if (!write_pgm(output_path, edges, width, height)) {
        fprintf(stderr, "Failed to write %s\n", output_path);
        free(input);
        free(edges);
        return 1;
    }

    printf("Wrote edges to %s (size: %zux%zu, threshold: %u)\n",
           output_path, width, height, threshold);

    free(input);
    free(edges);
    return 0;
}