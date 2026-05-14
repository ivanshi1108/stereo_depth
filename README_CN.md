# Stereo Depth 运行与参数说明

本文档为运行、参数和功能说明。

- 编译和安装命令：见 [`README.md`](README.md)

## 概述

本示例可接入双目 UVC 相机，也支持将 YUYV/MCAP 文件作为输入源，完成双目深度推理，并发布或录制以下结果：

- RGB 图像
- 视差图
- 点云
- 网格平均距离图像
- ROI z 均值摘要；无论是实时输入还是文件注入输入，都可选附带 UART 激光测距值

程序支持两种推理后端：

- NPU
- DSP，支持单核或双核执行

程序启动后，主要提供以下能力：

- Foxglove 实时可视化
- 可选 HDMI VO 四分屏预览，不依赖 Foxglove
- MCAP 录制与单帧导出，并可选附带 H.264 `CompressedVideo`
- 终端实时性能看板，可持续观察各处理阶段的吞吐、耗时、丢帧和内存占用

## 关键行为

- 输入既可以来自实时 UVC 采集，也可以通过 `-i` 指定 `.yuyv` / `.mcap` 文件
- 支持导入 MCAP 文件：文件中需要包含有效的 `/camera/yuyv` 消息；或者在使用 `--mcap-stream h264` 时，包含有效的 `/camera/h264` `CompressedVideo` 消息
- 原始 `.yuyv` 注入和 MCAP `/camera/yuyv` 回放固定以 5 fps 运行
- MCAP `--mcap-stream h264` 回放会根据录制帧时间戳解码并送入流水线
- 如果导入的 MCAP 含有 `/camera/device_info`，会使用其中的 `serial_number` 进行标定查找
- GDC 支持默认 mesh，以及基于序列号的动态 mesh 生成
- Foxglove 会发布处理后的主题、描述当前输入源的 JSON `/camera/device_info` 主题，以及同时适用于实时 UVC 和注入输入的 `/camera/roi_z_avg`
- 示例还会从 `/dev/ttyUSB0` 以 `9600 8N1` 读取可选的外部激光测距仪，并将最新有效距离附加到每帧 ROI 摘要中
- 如果激光测距仪不存在或暂时不可读，主双目流水线仍会继续运行，激光距离字段会输出为 `null`
- 如果 `/dev/ttyUSB0` 之后重新出现，示例会自动重试打开并恢复发布有效的激光距离值
- 在注入图像文件模式下，Foxglove 还会发布原始 `/camera/yuyv` 输入图像

## 运行

### 最小运行方式

```bash
cd /root/stereo_depth/output/sample_stereo_depth
LD_LIBRARY_PATH="$PWD/lib:/soc/lib:$LD_LIBRARY_PATH" ./sample_stereo_depth
```

最小运行方式使用以下默认值：

- 输入源：UVC 相机
- 输入设备：`/dev/video0`
- 输入分辨率：默认 `2560x720`；支持的双目输入为 `2560x720` 和 `3840x1080`
- 输入帧率：`15 fps`
- 推理后端：`npu`
- NPU 模型：默认自动查找 `axstereo_pro.axmodel`
- GDC 模式：`on`
- Foxglove 端口：`8765`
- MCAP 输出前缀：`/tmp/stereo_depth_dump`

运行时相关默认查找顺序如下：

- NPU 模型：`STEREO_DEPTH_MODEL_DIR` -> 可执行文件同级 `models/` -> 源码目录 `models/` -> 旧路径 `/opt/bin/sample_stereo_depth/models/`
- DSP 固件：`STEREO_DEPTH_DSP_DIR` -> 可执行文件同级 `dsp/` -> 源码目录 `third_party/dsp` -> 旧路径 `/opt/data/dsp`

### 命令行参数

```text
-d <device>    UVC 设备，默认：/dev/video0
-s <WxH>       双目分辨率：2560x720 或 3840x1080
-w <width>     双目宽度，默认：2560；FHD 输入时使用 3840
-h <height>    双目高度，默认：720；FHD 输入时使用 1080
-f <fps>       UVC 帧率，默认：15
-F <fps>       Foxglove 发布最大 fps，默认：15；0 表示不限制
-r <path>      MCAP 录制前缀或目录，默认：/tmp/stereo_depth_dump
-l             列出支持的 UVC 分辨率和帧率后退出
--uvc-list-all-controls          列出设备暴露的全部 V4L2 控件后退出
--uvc-reset-controls             将可写 UVC 控件重置为驱动默认值后退出
-i <file>      使用指定的 .yuyv 或 .mcap 文件替代 UVC 采集
--mcap-stream <mode> 选择 MCAP 导入源：yuyv | h264，默认：yuyv
-e <engine>    推理后端：npu | dsp，默认：npu
-m <model>     NPU 模型路径，默认：自动查找 axstereo_pro.axmodel
-g <gdc>       GDC 模式：on | force | builtin | off，默认：on
-c <core>      DSP 核模式：dual | single | 2 | 1，默认：dual
-q <depth>     每个客户端的 Foxglove 消息积压队列大小，默认：10
-t             打开性能 trace 日志
--vo                           启用 HDMI VO 四通道预览
--uvc-brightness <n>             设置 V4L2_CID_BRIGHTNESS
--uvc-contrast <n>               设置 V4L2_CID_CONTRAST
--uvc-saturation <n>             设置 V4L2_CID_SATURATION
--uvc-gamma <n>                  设置 V4L2_CID_GAMMA
--uvc-sharpness <n>              设置 V4L2_CID_SHARPNESS
--uvc-white-balance-auto <bool>  设置 V4L2_CID_AUTO_WHITE_BALANCE
--uvc-white-balance-temperature <n> 设置 V4L2_CID_WHITE_BALANCE_TEMPERATURE
--uvc-power-line-frequency <mode> 设置 V4L2_CID_POWER_LINE_FREQUENCY: off | 50 | 60
--uvc-gain <n>                    设置 V4L2_CID_GAIN
```

### 逆光场景调优

如果背景很亮、人物偏暗，建议优先调节 UVC 相机控件，而不是直接修改双目深度流水线本身。

具体可用控件取决于相机型号。以 `--uvc-list-all-controls` 输出中的 ZED 设备为例，当前示例直接开放了这些可调参数：`brightness`、`contrast`、`saturation`、`gamma`、`gain`、`sharpness`、`white_balance_auto`、`white_balance_temperature` 和 `power_line_frequency`。相机也可能上报其他控件，例如 hue；有些控件在关闭相关自动模式前，可能会显示为 `inactive`。

- 用 `--uvc-gamma` 和 `--uvc-brightness` 提亮前景；如果高光过强，再适当降低 `--uvc-contrast`
- 之后再增加 `--uvc-gain`，因为增益也会带来更多噪声

示例在启动时会查询相机上报的控件范围，对越界或不符合步进的值自动修正，并记录实际应用的值。如果某个控件当前为 `inactive`，示例会明确提示它被其他自动设置锁定，而不是静默地将其视为不支持。

在调参前，如果想查看相机特定的支持范围，请执行：

```bash
./sample_stereo_depth --uvc-list-all-controls
```

该输出还会显示控件是否由示例直接暴露（`sample=yes|no`），以及当前是否可调（`adjustable_now=yes|no`）。这也有助于检查诸如亮度、gamma、白平衡或厂商自定义 HDR/WDR 相关控件等非标准项。

如果要将示例支持的可写控件恢复为驱动上报的默认值，请执行：

```bash
./sample_stereo_depth --uvc-reset-controls
```

### HDMI VO 预览

使用 `--vo` 启用 HDMI 预览，无需依赖 Foxglove 客户端。

当前 HDMI 布局为 `2x2` 四宫格：

- 通道 0：原始输入图像
- 通道 1：左目图像
- 通道 2：视差伪彩图
- 通道 3：带 ROI 深度和置信度叠加的 Z 均值块图

当前 VO 输出会初始化 HDMI 为 `1080P60`。

### GDC 模式

- `-g on`：启用 GDC，先读取输入序列号；若本地已有 INI 和 mesh 则复用，仅在需要时下载和生成
- `-g force`：启用 GDC，INI 处理逻辑与 `on` 相同，但在 INI 就绪后总是重新生成 mesh
- `-g builtin`：启用 GDC，并使用内置默认 mesh 文件和内置默认 baseline
- `-g off`：关闭 GDC，适用于流水线调试

### 标定资源与回退逻辑

- 标定 INI：`zed_<serial>.ini`
- Mesh 文件：`zed_<serial>_<resolution>_l_<width>x<height>.txt`、`zed_<serial>_<resolution>_r_<width>x<height>.txt`

动态 mesh 生成会根据输入分辨率选择标定分段：

- `2560x720` 双目输入使用 `HD` 标定参数
- `3840x1080` 双目输入使用 `FHD` 标定参数
- 示例会记录标定 INI 路径、选择的分辨率以及用于生成 mesh 的 INI 键值

`builtin` 模式使用以下 mesh 文件和内置默认 baseline：

- `/opt/data/npu_disp/mesh_left.txt`
- `/opt/data/npu_disp/mesh_right.txt`

当示例下载或刷新动态标定资源时，会自动创建 `zed/` 目录。

`-g on` 和 `-g force` 的运行时行为如下：

1. 先读取输入序列号；如果拿不到序列号，则回退到 `builtin` 模式。
2. 如果有序列号，优先在本地查找 `zed_<serial>.ini`。仅当本地缺失时才下载；若下载仍失败，则回退到 `builtin` 模式。
3. INI 就绪后，优先查找对应 mesh 文件。如果两个 mesh 文件都存在，就连同标定 `Baseline` 一起复用；否则重新生成 mesh 文件，然后连同标定 `Baseline` 一起使用。
4. 在 `force` 模式下，只要 INI 就绪，就会将 mesh 查找视为未命中，并无条件重新生成 mesh 文件。

如果 mesh 生成失败，原因可能是 INI 不完整、缺少当前输入几何所需的分辨率分段或双目标定键值，或者无法被正确解析。此时示例会记录 warning，并回退到内置 mesh 和 baseline，而不是中止启动。

如果有效的标定 INI 中包含 `Baseline`，在 `-g on` 和 `-g force` 模式下会在运行时加载该值，并从毫米转换为米。

## 输入模式

### UVC 采集

```bash
./sample_stereo_depth -d /dev/video0 -w 2560 -h 720 -f 15
```

该方式使用实时 UVC 双目相机运行，并使用检测到的设备序列号进行动态标定。

你也可以将采集分辨率作为单个参数传入，例如：

- `./sample_stereo_depth -d /dev/video0 -s 2560x720 -f 15`
- `./sample_stereo_depth -d /dev/video0 -s 3840x1080 -f 15`

```bash
./sample_stereo_depth -d /dev/video0 -w 2560 -h 720 -f 15
```

该方式使用实时 HD UVC 输入。在启用 GDC 时，流水线会先拆分原始双目图像，再将左右目分别直接去畸变到 `640x384`。动态 mesh 生成会使用 `HD` 标定参数。

```bash
./sample_stereo_depth -d /dev/video0 -w 3840 -h 1080 -f 15
```

该方式使用实时 FHD UVC 输入。在启用 GDC 时，流水线会先拆分原始双目图像，再将左右目分别直接去畸变到 `640x384`。动态 mesh 生成会使用 `FHD` 标定参数。

### 原始 YUYV 文件注入

```bash
./sample_stereo_depth -i /opt/data/npu_disp/zed_2560x720.yuyv -F 5
```

该模式会以固定 5 fps 重复注入单帧原始 YUYV 数据。适合在无相机可用时进行离线验证。

对于其他支持的输入分辨率，导入原始文件时请传入匹配的 `-w` 和 `-h`，例如：

- `./sample_stereo_depth -i /opt/data/npu_disp/zed_2560x720.yuyv -w 2560 -h 720 -F 5`
- `./sample_stereo_depth -i /opt/data/npu_disp/zed_3840x1080.yuyv -w 3840 -h 1080 -F 5`

### MCAP 文件注入

```bash
./sample_stereo_depth -i /tmp/stereo_depth_dump_single_123456789.mcap
```

该模式会解析 MCAP 文件，并从 `/camera/yuyv` 或 `/camera/h264` 中回放输入。

- 默认导入模式是 `yuyv`，直接读取 `/camera/yuyv`
- `--mcap-stream h264` 会读取 `/camera/h264`，使用 VDEC 解码录制的 H.264 访问单元，并在旧的 YUYV->NV12 转换阶段之后，直接将解码得到的 NV12 帧缓冲送入双目流水线，这样 GDC 就能复用 VDEC buffer，避免额外一次 NV12 拷贝
- 在 `yuyv` 模式下，应用从第 `1/N` 帧开始，以固定 5 fps 运行所选帧；可以在终端中按左右方向键切换到上一帧或下一帧
- 在 `h264` 模式下，应用会按录制时间戳顺序回放帧；该模式下不支持左右方向键选帧

该模式支持回放含有有效 `/camera/yuyv` 帧或兼容 `/camera/h264` 录制数据的 MCAP 文件，包括单帧导出文件、兼容录制文件，以及持续追加的多帧导出文件。

MCAP 导入规则：

- `/camera/yuyv` 必须使用 `yuyv` 编码
- `--mcap-stream h264` 要求 `/camera/h264` 为有效的 Foxglove `CompressedVideo` 消息，且 `format = h264`
- `.mcap` 输入会忽略 `-w` 和 `-h`
- `/camera/device_info` 必须声明 `width` 和 `height`；否则该 MCAP 文件会被视为无效
- 在导入原始帧时，`/camera/device_info` 中声明的 `width` 和 `height` 必须与 `/camera/yuyv` 的真实分辨率一致
- 仅支持 `2560x720` 和 `3840x1080` 的 MCAP 双目输入
- 会统计所选 MCAP 流中的全部有效导入消息；仅 `/camera/yuyv` 回放支持交互式左右切帧
- 如果存在 `/camera/device_info`，会使用其中的 `serial_number` 做动态标定查找
- 如果存在 `/camera/roi_z_avg`，回放 MCAP 时会复用录制的 `laser_distance_mm` 值，不再读取 `/dev/ttyUSB0`。为兼容旧版本，回放也接受仍使用 `laser_distance_m` 的历史录制文件
- 同一文件中的所有有效 `/camera/yuyv` 消息必须使用相同分辨率
- H.264 导入会跳过前导的、无法启动可解码序列的非 IDR 访问单元，然后从首个可解码点开始按录制时间戳继续回放

## 常用示例

- `./sample_stereo_depth -e npu`

  使用 NPU 后端运行。

- `./sample_stereo_depth -e npu -m /opt/bin/sample_stereo_depth/models/custom.axmodel`

  使用自定义 NPU 模型运行。

- `./sample_stereo_depth -e dsp`

  使用 DSP 双核模式运行。

- `./sample_stereo_depth -e dsp -c 1`

  使用 DSP 单核模式运行。

- `./sample_stereo_depth -g builtin`

  启用 GDC，但跳过基于序列号的动态 mesh 生成。

- `./sample_stereo_depth -g force`

  强制重新生成基于序列号的 mesh 文件。

- `./sample_stereo_depth -F 10`

  将 Foxglove 发布帧率限制为 10 fps。

- `./sample_stereo_depth --vo`

  启用当前四通道布局的 HDMI VO 预览。

- `./sample_stereo_depth -r /root/`

  将录制文件保存到 `/root/`，使用默认基础名称 `stereo_depth_dump`。

- `./sample_stereo_depth -r /root/my_capture`

  使用 `/root/my_capture` 作为录制文件前缀。

- `./sample_stereo_depth -l`

  读取相机声明的 UVC 模式，只打印本示例支持的分辨率：`2560x720` 和 `3840x1080`。

- `./sample_stereo_depth --uvc-list-all-controls`

  打印当前 UVC 设备暴露的全部 V4L2 控件，包括范围、步进、默认值和当前值。

- `./sample_stereo_depth --uvc-reset-controls`

  按安全依赖顺序将示例支持的可写 UVC 控件恢复为驱动上报的默认值。

- `./sample_stereo_depth --uvc-white-balance-auto off --uvc-white-balance-temperature 4600`

  关闭自动白平衡，然后设置手动白平衡色温。

- `./sample_stereo_depth --uvc-power-line-frequency 50`

  在 50 Hz 市电环境中设置防闪烁模式。

- `./sample_stereo_depth --uvc-gamma 180 --uvc-brightness 6 --uvc-contrast 3`

  当曝光控制不可用或受限时，在逆光场景下通过图像控件提亮偏暗前景。

- `./sample_stereo_depth --uvc-gamma 180 --uvc-brightness 8 --uvc-gain 5`

  一个更适合 ZED 的逆光场景示例，只使用该设备清单中暴露的控件。

- `./sample_stereo_depth -q 3`

  将每个客户端的 Foxglove 消息积压队列限制为 3，以减少客户端消费较慢时的内存占用。

- `./sample_stereo_depth -t`

  启用更详细的 trace 日志。

## 激光测距仪

该示例可以为每一帧采集图像关联一个外部 UART 激光距离值。

- 默认设备：`/dev/ttyUSB0`
- UART 配置：`9600 8N1`
- 期望响应帧：`01 03 04 <4-byte distance> <2-byte CRC16-RTU>`
- CRC：Modbus RTU 多项式 `0xA001`，在线路上传输时 CRC 低字节在前
- 距离换算：4 字节载荷按大端无符号整数解析，单位为 `0.0001 m`，发布和录制前再转换为 `mm`

当前关联逻辑刻意保持简单：

- 每次采集到相机帧时，运行时会尝试读取最新可用的激光数据包
- 如果当前帧没有新包可读，运行时会临时复用最近一次有效距离样本
- 当前不会在历史缓冲中搜索时间上最接近的激光样本

## Foxglove 可视化

程序启动后会监听 `8765` 端口。将 Foxglove Studio 连接到开发板 IP 地址后，即可查看实时结果。

发布的主题包括：

- `/camera/device_info`
- `/camera/roi_z_avg`
- `/camera/rgb`
- `/camera/disparity`
- `/camera/pointcloud`
- `/camera/z_grid_avg`
- `/camera/grid`

当输入源是注入的 `.yuyv` 或 `.mcap` 文件时，Foxglove 还会发布：

- `/camera/yuyv`

主题含义：

- `/camera/device_info`：当前输入源的 JSON 元数据，包括 `input_mode`、`device`、`serial_number`、`is_usb`、`width`、`height` 和 `fps`
- `/camera/yuyv`：原始注入的 YUYV 帧；对于注入的 `.yuyv` 和 `.mcap` 输入会实时发布
- `/camera/roi_z_avg`：逐帧 ROI 摘要 JSON。其 `items` 数组以 `{ "name": "laser_distance_mm", "z_avg_mm": <value-or-null> }` 开头，后面依次是 3×3 排布的 9 个 ROI 采样点，固定像素位置位于 640×384 深度图上的 X: 180/320/460、Y: 96/192/288；每个 ROI 条目都包含 `z_avg_mm` 和 `confidence_avg`，基于周围 11×11 窗口计算。该主题会对 UVC 和注入输入实时发布，也会写入 MCAP
- `/camera/rgb`：转换为 RGB 的左目输入图像
- `/camera/disparity`：视差结果，范围 `0-2048`
- `/camera/pointcloud`：`x/y/z + alpha/r/g/b` 格式的点云
- `/camera/z_grid_avg`：基于网格的平均距离图像
- `/camera/grid`：网格叠加标注

## MCAP 录制

程序运行期间，可以通过键盘控制录制：

```text
按 r 开始录制
再次按 r 停止录制
按 d 将单帧导出到本次运行对应的单帧 MCAP 文件
再次按 d 会继续向同一个单帧 MCAP 文件追加一帧
H.264 编码器会在启动阶段完成初始化
每次程序运行中第一次按 d 时，也会同时启动基于原始 YUYV 输入的连续 H.264 硬件编码录制
连续 H.264 流会以 Foxglove `CompressedVideo` 消息写入同一个 `_single.mcap` 文件，因此 Foxglove 打开该 dump MCAP 时可直接播放
通过 r 开始的常规录制，在 VENC 可用时也会在同一个 MCAP 文件中写入 H.264 `CompressedVideo` 消息
```

这些键盘控制仅在程序运行于 TTY 时可用。如果 stdin 不是 TTY，则会禁用键盘录制控制。

生成文件名遵循以下规则：

- 目录形式：`-r /root/` -> `/root/stereo_depth_dump[_sn<serial>]_<timestamp>.mcap`
- 前缀形式：`-r /root/my_capture` -> `/root/my_capture[_sn<serial>]_<timestamp>.mcap`
- 默认：`/tmp/stereo_depth_dump[_sn<serial>]_<timestamp>.mcap`
- 单帧导出：每次程序运行中第一次按 `d` 会创建 `/tmp/stereo_depth_dump_single[_sn<serial>]_<timestamp>.mcap`（或对应的自定义前缀形式），之后再按 `d` 会继续向同一文件追加更多帧

不带 `_single` 后缀的是通过 `r` 启动的常规录制文件；带 `_single` 后缀的是通过 `d` 生成的、本次运行对应的单帧导出文件。
每次程序运行中第一次按下 `d` 后，还会开始向同一个 `_single` MCAP 文件持续追加 H.264 `CompressedVideo` 帧，直到程序退出。

无论是通过 `r` 触发的常规录制，还是通过 `d` 触发的单帧导出录制，当当前 MCAP 文件达到 `512 MiB` 时，都会自动滚动到下一个分段文件。第一段保留原始文件名，后续分段会追加 `_part02`、`_part03` 等后缀。

如果当前输入源能提供序列号，文件名中会自动附加序列号后缀。

常规 MCAP 录制始终包含 `/camera/device_info`，以及当前帧对应的处理结果输出。当 VENC 可用时，常规录制还会包含以 Foxglove `CompressedVideo` 形式写入的 `/camera/h264`。写入 MCAP 的 `/camera/roi_z_avg` 消息中，也会包含前文提到的激光距离字段。当当前输入源是注入的 `.yuyv` 或 `.mcap` 文件时，常规录制还会把原始输入 YUYV 帧写成 `/camera/yuyv`。每次运行对应的单帧导出文件则始终包含 `/camera/yuyv`；如果 VENC 可用，还会同时包含 `/camera/h264`，因此后续可以直接通过 `-i` 再次导入使用。默认回放模式下，重放注入的 `.mcap` 文件会使用 `/camera/yuyv`；使用 `--mcap-stream h264` 时则会改为使用 `/camera/h264`。无论哪种方式，回放期间 Foxglove 都会实时发布 `/camera/yuyv` 和 `/camera/roi_z_avg`。

## 性能看板

程序运行过程中，终端会按 1 秒滚动刷新一次性能看板，用于观察各阶段的实时吞吐和延迟表现。

当前会统计以下阶段：

- `cap`：采集或导入输入帧
- `pre`：预处理，包括格式转换、切分、GDC、缩放等前处理操作
- `infer`：深度推理执行阶段
- `post`：后处理，例如结果整理、点云或统计图生成等
- `pub`：Foxglove 主题发布
- `dump`：MCAP 录制、单帧导出及相关写盘路径

看板中会显示以下指标：

- `FPS`：该阶段当前吞吐速率
- `wait_in_ms`：该阶段等待输入数据或等待上游结果的时间
- `work_ms`：该阶段实际执行工作的耗时
- `dropped`：该阶段累计或当前窗口内的丢帧情况，用于观察是否存在处理跟不上的问题
- `e2e_ms`：从输入到该阶段完成时的端到端耗时
- `rss_mb`：当前进程驻留内存大小，单位为 MB
- `maximum queue depth`：运行过程中观测到的最大队列深度，可用于判断是否存在明显堆积

这个看板适合用于快速判断瓶颈大致落在哪个阶段。例如：

- 如果 `cap` 或 `pre` 的 `wait_in_ms` 很高，通常表示输入侧节奏较慢，或上游供数不稳定
- 如果 `infer` 的 `work_ms` 明显偏高，往往说明推理阶段是当前吞吐瓶颈
- 如果 `pub` 或 `dump` 的队列深度持续升高，通常意味着发布或写盘速度跟不上处理速度
- 如果 `dropped` 持续增加，则说明整条链路已经开始丢帧，需要进一步降低负载或缩减输出

如需查看更多调试细节，可配合 `-t` 打开更详细的 trace 日志。