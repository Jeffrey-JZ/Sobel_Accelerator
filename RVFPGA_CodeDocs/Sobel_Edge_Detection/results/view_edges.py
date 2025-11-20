# Visualise and reconstruct edge detection data
# Author: Junze Jiang
# 17/11/2025

"""

For upload(not have debug step):
    For visualisation: python -m pip install numpy matplotlib
    First, check how many serial ports are currently active; we only require COM4

    In the VS Code terminal, type:
        pio device list
        pio device monitor --port COM4 --baud 115200 > edges_raw.txt
    
    Keep this terminal open with the command running 
    "upload" the programme again to run the Sobel programme again

    Once it has printed all the numbers on the entire image (10s), press Ctrl + C in the terminal to terminate the pio device monitor

    Target: edges_raw.txt

    Rename this file to "edge_bump"
    Move it to the results directory

    cd results python
    .\view_edges.py

    Attention: The file of "edge_bump" is UTF-16 LE. Change it to UTF-8 and re-run the program

"""

import numpy as np
import matplotlib.pyplot as plt

W, H = 128, 128   # Change it to match IMG_W / IMG_H.

values = []
with open("edges_dump.txt", "r", encoding="utf-8", errors="ignore") as f:
    for line in f:
        line = line.strip()
        if not line:
            continue
        # Collect all elements within a line that can be converted into integers
        for p in line.split():
            try:
                values.append(int(p))
            except ValueError:
                pass

print("Total number of pixels read:", len(values))

# Extract only the first W × H pixels, then reshape into a two-dimensional array of size H × W.
arr = np.array(values[:W*H], dtype=np.uint8).reshape(H, W)

plt.imshow(arr, cmap="gray")
plt.title("Sobel edges from FPGA")
plt.axis("off")
plt.show()
plt.imsave("edges_from_fpga.png", arr, cmap="gray")
