##Using the Sobel Operator in a PlatformIO Project

This example splits the Sobel edge detection implementation into `sobel.c` and `sobel.h`, facilitating direct copying into the `src/` directory under PlatformIO. This represents the minimal integration steps:

1. Copy `sobel.c` and `sobel.h` to your PlatformIO project's `src/` directory. The source code relies solely on C runtime dependencies (`<stdint.h>`, `<stddef.h>`, `<stdlib.h>`) and does not depend on dynamic memory.
2. Include the header files and call `sobel_edge` in your application's `main.c`:

```c
#include "sobel.h"

#define IMG_W 128
#define IMG_H 128

static uint8_t input[IMG_W * IMG_H]； // Place greyscale image
static uint8_t edge[IMG_W * IMG_H]； // Sobel result buffer

void app_run(void)
{
    const uint8_t 阈值 = 60; // Adjust according to the noise environment
    sobel_edge(input, edges, IMG_W, IMG_H, threshold);
}
```

3. Should your platform lack `printf`, simply utilise the aforementioned `sobel.c` / `sobel.h` files; `sobel_demo.c` serves solely as a PC-side verification example and need not be replicated.
4. When employing an FPGA soft core (such as RVFPGA) with UART output, iterate through `edges` within `app_run` to output results via your existing serial port/display driver.
5. The threshold may be dynamically adjusted based on lighting or noise; for stronger suppression, replace `magnitude > threshold` with `magnitude >= threshold`.

> Note: The algorithm employs register accumulation only, making it suitable for execution on soft cores without an FPU. Ensure intermediate accumulation uses `int` type, and when increasing resolution, verify input pixels remain within the 0–255 range.
> 
##How does one feed a 'photograph' into the Sobel operator?

The photograph must first be converted into a grayscale pixel array before being supplied to `sobel_edge` via the serial port:

- **Offline conversion to C database**: On the PC, use Python/Pillow to convert photographs to 8-bit greyscale, resize them to the appropriate resolution, then export them to a C database for compilation into the hardware.
 

  ```python
  # img_to_array.py
  

  img = Image.open("input.jpg").convert("L").resize((128, 128))
  data = np.asarray(img, dtype=np.uint8)
  with open("image_data.h", "w") as f:
      f.write("#pragma once\n")
      f.write("#define IMG_W 128\n#define IMG_H 128\n")
      f.write("static const unsigned char input[IMG_W*IMG_H] = {\n")
      for idx, v in enumerate(data.flatten()):
          end = ","\n" if (idx + 1) % 16 == 0 else ","
          f.write(f" {v}{end}")
      f.write("\n};\n")
  ```

  Then, in `main.c`, include `#include "image_data.h"` and pass `input` to `sobel_edge`.

- **Input during runtime**: If your platform possesses input channels such as a camera, card, or serial port, read the grayscale data within the driver and populate the `input` buffer. Subsequently, invoke `sobel_edge`. Simply ensure the resolution, grayscale size, and threshold align with your hardware specifications.
 

To test directly with real photographs on the PC, you may compile and run `sobel_pgm_demo.c`:

```bash
gcc sobel_pgm_demo.c sobel.c -o sobel_pgm_demo
./sobel_pgm_demo input.pgm output.pgm 80
```

## The Complete Process for Board Setup and Debugging on PlatformIO

Below, using VS Code with the PlatformIO extension or the command-line tool `pio` as an example, we demonstrate integrating the Sobel filter into an FPGA soft core project and proceeding with download/debugging:

1) **Create/Initialise Project**

```bash
# Choose one: Create a new project in VS Code using PlatformIO Home;
# or initialise via command line (using RVfpga VexRiscv as an example; replace with your board ID):
pio project-initialise --board sifive-hifive1-revb --project-option "platform=riscv" \
    --project-option "framework=arduino" # or baremetal/other frameworks
```

2) **Copy algorithm source code**

- Copy `sobel.c` and `sobel.h` to the `src/` directory of the new project.
- If assuming a single image is fixed into the firmware, first run `image_to_matrix.py` to generate `image_data.h`, also placing it in `src/`.

3) **Add minimal `main.c`**

You may directly use `examples/platformio_main.c` from the repository as a template, copying it to `src/main.c`:

```bash
cp RVFPGA_CodeDocs/examples/platformio_main.c /path/to/your/pio-project/src/main.c
```

`platformio_main.c` invokes `sobel_edge` to process the grayscale clusters from `image_data.h`, outputting results line-by-line via `printf`. Should your board feature a semi-host/standard output, you may adapt the output section to utilise custom drivers for UART, LCD, SD card, etc.

4) **Configuring the Serial/Debug Port**

- Set the appropriate upload port and convenient baud rate in `platformio.ini`, for example:

```ini
upload_port = /dev/ttyUSB0
monitor_speed = 115200
```

- If using JTAG/OpenOCD, configure fields such as `debug_tool` and `upload_protocol` according to your board's documentation.

5) **Compilation and Download**

```bash
pio run # Compile
pio run -t upload # Download to FPGA soft core via serial port/JTAG
```

6) **Viewing Results via Serial Port**

```bash
pio device monitor # Open serial port to view Sobel output matrix
```
