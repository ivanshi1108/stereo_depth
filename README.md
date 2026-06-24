# Stereo Depth 示例（AXCL版）

## 架构设计
![系统架构](res/pipeline.png)

## 已验证平台

| 硬件平台 | 操作系统 |
| --- | --- |
| Raspberry Pi 5 | Debian GNU/Linux 13 (trixie) |
| RK3588 OPi 5 Plus | Ubuntu 22.04.5 LTS |
| AMD64 PC | Ubuntu 22.04.4 LTS |

## 支持的双目模组

- ZED Mini Stereo Camera

## 编译

### 安装依赖

```bash
sudo apt update
sudo apt install -y build-essential cmake pkg-config \
    libopencv-dev libavcodec-dev libavutil-dev libswscale-dev libcurl4-openssl-dev
```

请确认已安装 AXCL 驱动 ，并用 `axcl-smi` 确认算力卡工作正常。

### 构建

在工程根目录执行：

```bash
make clean
make build
make install
```

- 编译输出：`build/`
- 安装输出：`output/sample_stereo_depth/`

## 运行

安装后目录结构：

```text
output/sample_stereo_depth/
├── sample_stereo_depth
├── lib/
└── models/
```

### 默认运行（UVC 采集 + 预览窗口 + foxglove 发布）

```bash
cd output/sample_stereo_depth
./sample_stereo_depth
```

### 运行效果
#### Raspberry Pi 5
https://github.com/user-attachments/assets/b6032c92-cc0a-46af-8198-5126754b5259
#### RK3588 OPi 5 Plus
https://github.com/user-attachments/assets/414acdc6-9565-4a1a-a619-64f3e12e51d3
#### AMD64 PC （Ubuntu）
https://github.com/user-attachments/assets/fe89d0e7-c142-49f3-b033-bca0b674f95e



### 其他常用操作
#### 回放 MCAP 文件

```bash
./sample_stereo_depth -i /path/to/your_dump.mcap
./sample_stereo_depth -i /path/to/your_dump.mcap --mcap-stream h264 # 回放录制里的 H.264 流
```

#### 不开预览窗口（无桌面预览，仅 foxglove 发布）

```bash
./sample_stereo_depth --no-vo -i /path/to/your_dump.mcap
```

#### --imgproc参数说明
- `--imgproc <host|axcl|auto>`：图像处理后端，**默认 `auto`**。
- `auto` / `axcl`：优先用算力卡 IVPS 做图像处理，失败时自动回退HOST CPU。
- `host`：全部用HOST CPU 执行图像处理。

## 详细说明

- 更完整的运行参数，请参阅 https://zhuanlan.zhihu.com/p/2047786127969477270。
