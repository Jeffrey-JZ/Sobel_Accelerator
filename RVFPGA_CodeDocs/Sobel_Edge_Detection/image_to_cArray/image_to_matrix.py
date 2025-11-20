# Convert an image into data usable by C language on a PC
# i.e. The matrix values of the image to be detected
# Author: Junze Jiang
# 17/11/2025

"""
Convert any image into:
1) A greyscale array directly usable in C code (header file: image_data.h)
2) A text matrix for easy viewing/debugging (optional)

    Place the image to be tested and image_to_matrix.py in the same directory
    python .\image_to_matrix.py --input .\lena_original.jpg --width 128 --height 128 --header-out .\image_data.h --txt-out .\image_matrix.txt --var-name input

"""

# Using Pillow to process the image
# Under Python3: pip install pillow
# In command window in vscode: python -m pip install pillow; Check: python -m pip show pillow
import argparse             # Parsing command-line arguments
from pathlib import Path    # Processing file paths
from typing import List

from PIL import Image

# Open the image and force conversion to greyscale ‘L’
def load_image(path: Path, width: int | None, height: int | None) -> tuple[int, int, List[int]]:
    img = Image.open(path).convert("L")
    if width is not None and height is not None:
        img = img.resize((width, height))
    w, h = img.size
    data = list(img.getdata())
    return w, h, data

# Output C head file: image_data.h
def write_header(outfile: Path, w: int, h: int, data: List[int], var_name: str) -> None:
    with outfile.open("w", encoding="utf-8") as f:
        f.write("#pragma once\n")
        f.write(f"#define IMG_W {w}\n#define IMG_H {h}\n")
        f.write(f"static const unsigned char {var_name}[IMG_W*IMG_H] = {{\n")
        for idx, val in enumerate(data):
            end = ",\n" if (idx + 1) % 16 == 0 else ","
            if idx + 1 == len(data):
                end = "\n"
            f.write(f" {val}{end}")
        f.write("};\n")

# Write pixels to text by row
def write_txt(outfile: Path, w: int, h: int, data: List[int]) -> None:
    with outfile.open("w", encoding="utf-8") as f:
        for y in range(h):
            row = data[y * w : (y + 1) * w]
            f.write(" ".join(f"{v:3d}" for v in row))
            f.write("\n")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Convert image to grayscale C array and optional matrix dump")
    parser.add_argument("--input", required=True, help="Source image path (jpg/png/bmp/pgm etc.)")                  # Origianl image
    parser.add_argument("--width", type=int, help="Resize width; keep original if omitted")                         # Target dimensions (128×128)
    parser.add_argument("--height", type=int, help="Resize height; keep original if omitted")                       
    parser.add_argument("--header-out", default="image_data.h", help="Output header file path")                     # Output .h
    parser.add_argument("--txt-out", help="Optional matrix text file path")                                         # Output matrix txt
    parser.add_argument("--var-name", default="input", help="C array symbol name")                                  # Generated C array name, default input
    return parser.parse_args()

# Read in → Convert to greyscale and scale → Output header file and optional text matrix, printing conversion details to the terminal
def main() -> None:
    args = parse_args()
    input_path = Path(args.input)
    w, h, data = load_image(input_path, args.width, args.height)
    header_path = Path(args.header_out)
    write_header(header_path, w, h, data, args.var_name)
    if args.txt_out:
        write_txt(Path(args.txt_out), w, h, data)
    print(f"Converted {input_path} -> {header_path} ({w}x{h})")
    if args.txt_out:
        print(f"Matrix dump saved to {args.txt_out}")


if __name__ == "__main__":
    main()