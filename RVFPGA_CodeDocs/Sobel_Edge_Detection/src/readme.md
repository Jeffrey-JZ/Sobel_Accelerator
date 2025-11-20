# 在 PlatformIO/FPGA 工程中使用 Sobel 算子

本示例把Sobel边缘检测实现拆成`sobel.c` + `sobel.h` ，方便直接复制到PlatformIO的下面`src/`目录中。是最小化集成步骤：

1.把`sobel.c`和`sobel.h`复制到你的PlatformIO项目`src/`目录。源码只有C运行时依赖（`<stdint.h>` 、`<stddef.h>` 、`<stdlib.h>` ），不依赖动态内存。 
2.在你的应用`main.c`中包含头文件并调用`sobel_edge` ：

```c
#include "sobel.h"

#define IMG_W 128
#define IMG_H 128

静态uint8_t输入[IMG_W * IMG_H]； // 放置灰度图（0~255）
静态 uint8_t 边缘[IMG_W * IMG_H]； // Sobel 结果缓冲

void app_run(void)
{
    const uint8_t 阈值 = 60; // 根据噪声环境调整
    sobel_edge(input, edges, IMG_W, IMG_H, threshold);
    // 把边站显示/仓库/劳动力劳动力，由具体硬件决定
}
```

3.如果你的平台没有`printf` ，只需使用上面的`sobel.c` / `sobel.h`即可；`sobel_demo.c`只是PC端验证示例，不需要复制。
4.若使用FPGA软核（如RVFPGA）搭配UART输出，可以在`app_run`里遍历`edges` ，通过您现有的串口/显示驱动结果输出。
5.阈值可根据光线或噪声动态调整；如果需要更强的抑制，可以把`magnitude >threshold`替换为`magnitude >=threshold` 。

> 提示：算法只使用寄存器累加，适合在无 FPU 的软核上运行。中间注意使用 `int` 累加，分辨率增加时请保证输入像素不超出 0~255。

## 如何把「照片」送入 Sobel 算子？

FPGA 上没有「上传照片」的概念，你需要先把照片转换成层次像素阵列，再以串口/Flash/摄像头接口供给`sobel_edge` ：

- **离线转换为 C 数据库（最常用）** ：在 PC 端使用 Python/Pillow 将照片转成 8 位灰度、缩放到合适的分辨率，再导出为 C 数据库，编译进硬件。
 

  ```python
  # img_to_array.py
  从 PIL 导入图像
  导入 numpy 库并将其命名为 np

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

  然后在`main.c`中`#include "image_data.h"` ，把`input`传给`sobel_edge`即可。

- **运行时输入** ：如果你的平台有摄像头/卡/串口等输入通道，就在驱动里读取把灰度数据填入`输入`缓冲，再调用`sobel_edge` 。只要保证分辨率、灰度大小和阈值符合你的硬件就行。
 

PC端如果想直接用真实照片测试，可以编译运行`sobel_pgm_demo.c` ：

```bash
gcc sobel_pgm_demo.c sobel.c -o sobel_pgm_demo
./sobel_pgm_demo input.pgm output.pgm 80
```

它读取P5（二进制）PGM灰度图，输出同分辨率的边缘图，方便您先在电脑上验证效果再移植到FPGA。

## 我只有一张图片，如何获得矩阵/C 阵列？

仓库新增了一个`image_to_matrix.py` ，可以把`lenna`这样的照片直接变成Sobel可以使用的灰度矩阵或C数据库：

```bash
# 1) 安装 Pillow（如未安装）
pip install pillow

# 2) 将图片转成 128x128 灰度 C 阵列，同时输出可阅读的矩阵文本
python3 image_to_matrix.py --input lenna.png --width 128 --height 128 \
    --header-out image_data.h --txt-out image_matrix.txt --var-name input

# 3) 在 main.c 中包含并调用
#include "sobel.h"
#include "image_data.h" // 里面包含 input[IMG_W*IMG_H]

static uint8_t edges[IMG_W * IMG_H];

int main(void) {
    sobel_edge(input, edges, IMG_W, IMG_H, 60);
    // TODO: 根据硬件把边缘打印/显示/存储
    返回 0；
}
```

运行后会得到：

- `image_data.h` ：可直接编译到硬件里的云计算（`input` ）。
 
- `image_matrix.txt` ：纯文本矩阵，每行对应一行像素，由你确认数值或手工处理。
 

如果你想先在PC上验证效果，可以把同一张图片（或脚本生成的PGM）替换`sobel_pgm_demo` ：

```bash
python3 image_to_matrix.py --input lenna.png --width 128 --height 128 --header-out /tmp/lenna.h
转换 lenna.png -resize 128x128 -colorspace Gray /tmp/lenna.pgm # ImageMagick 或其他工具
gcc sobel_pgm_demo.c sobel.c -o sobel_pgm_demo
./sobel_pgm_demo /tmp/lenna.pgm /tmp/lenna_edges.pgm 80
```

这样您就获得了可烧录的集群，从而生成了边缘结果对比效果。

## PlatformIO上板与调试的完整流程

下面以 VS Code + PlatformIO 扩展或命令行`pio`为例，实例将 Sobel 集成到 FPGA 软核工程并下载/调试：

1) **创建/初始化工程**

```bash
# 任选其一：在 VS Code 里用 PlatformIO Home 创建新工程；
# 或在命令行初始化（以RVfpga VexRiscv为例，替换成你的板卡ID）：
pio 项目初始化 --board sifive-hifive1-revb --project-option "platform=riscv" \
    --project-option "framework=arduino" # 或 baremetal/其他框架
```

2) **复制算法源码**

-将`sobel.c` 、`sobel.h`复制到新工程的`src/`目录。
-若假设单张图片固化进固，先运行`image_to_matrix.py`生成`image_data.h` ，同样放入`src/` 。

3) **添加最小`main.c`**

你可以直接使用仓库中的`examples/platformio_main.c`作为模板，将其复制到`src/main.c` ：

```bash
cp RVFPGA_CodeDocs/examples/platformio_main.c /path/to/your/pio-project/src/main.c
```

`platformio_main.c`会调用`sobel_edge`处理`image_data.h`中的灰度集群，并通过`printf`按行输出结果；如果你的板卡半主机/标准输出，可以把输出部分改为UART、LCD、SD卡等自定义驱动。

4) **配置串口/调试端口**

-在`platformio.ini`中设置合适的上传端口与方便波特率，例如：

```ini
上传端口 = /dev/ttyUSB0
monitor_speed = 115200
```

-如果使用JTAG/OpenOCD，请根据板卡文档填写好`debug_tool` 、`upload_protocol`等字段。

5) **编译与下载**

```bash
pio 运行 # 编译
pio run -t upload # 通过串口/JTAG 下载到 FPGA 软核
```

6) **串口查看结果**

```bash
pio device monitor #打开串口，查看Sobel 输出矩阵
```

7) **在线调试（任选）**

若板卡支持硬件调试器（如JTAG），可使用：

```bash
pio run -t debug #启动调试会话，设置断点查看input/edges缓冲
```

> 提示：如果你的输入来自摄像头或外设，只需在 `main.c` 中用驱动填充 `input` 缓冲，调用 `sobel _edge` 后按你的方式把 `edges` 输出即可，算法本身无依赖、无动态分配，适合裸机环境。