# Stereo Depth 示例

## 操作示例


本视频示例使用的运行环境：

- 开发板：[爱芯派 / Maix-IV M4N Dock](https://wiki.sipeed.com/hardware/zh/maixIV/m4ndock/m4ndock.html)
- 
- 系统镜像：[AX650_pipro_box_ubuntu_rootfs_desktop_V3.10.2_20260610103629.axp](https://huggingface.co/AXERA-TECH/BoardImages/blob/main/AX650_pipro_box_ubuntu_rootfs_desktop_V3.10.2_20260610103629.axp)

编译及运行


https://github.com/user-attachments/assets/fef81d10-23aa-491c-8f4a-6cdc03a81588



HDMI0 输出效果



https://github.com/user-attachments/assets/42bdc123-7469-4e46-9c3e-ef4045ccf63d



## 操作命令
### 编译

在工程根目录执行：

```bash
cd /root/stereo_depth
make clean
make build
make install
```

编译输出目录：

- `build/`

安装输出目录：

- `output/sample_stereo_depth/`

### 运行

安装后的目录结构如下：

```text
output/sample_stereo_depth/
├── sample_stereo_depth
├── dsp/
├── lib/
└── models/
```

运行时需要把安装目录里的 `lib/` 和平台库目录 `/soc/lib` 加进 `LD_LIBRARY_PATH`。

#### 默认运行

```bash
cd /root/stereo_depth/output/sample_stereo_depth
LD_LIBRARY_PATH="$PWD/lib:/soc/lib:$LD_LIBRARY_PATH" ./sample_stereo_depth
```

#### 开启 HDMI VO 预览

```bash
cd /root/stereo_depth/output/sample_stereo_depth
LD_LIBRARY_PATH="$PWD/lib:/soc/lib:$LD_LIBRARY_PATH" ./sample_stereo_depth --vo
```

#### 回放 MCAP 文件

```bash
cd /root/stereo_depth/output/sample_stereo_depth
LD_LIBRARY_PATH="$PWD/lib:/soc/lib:$LD_LIBRARY_PATH" \
./sample_stereo_depth --vo -i /root/your_dump.mcap
```

#### 只检查可支持的 UVC 模式

```bash
cd /root/stereo_depth/output/sample_stereo_depth
LD_LIBRARY_PATH="$PWD/lib:/soc/lib:$LD_LIBRARY_PATH" ./sample_stereo_depth -l
```

## 常见说明

- 默认 NPU 模型会优先从安装目录下的 `models/axstereo_pro.axmodel` 加载。
- 默认 DSP 固件会优先从安装目录下的 `dsp/` 加载。
- 如果板子当前没有可用的动态标定文件或默认 mesh，可以先用 `-g off` 排除 GDC 问题。

例如：

```bash
cd /root/stereo_depth/output/sample_stereo_depth
LD_LIBRARY_PATH="$PWD/lib:/soc/lib:$LD_LIBRARY_PATH" \
./sample_stereo_depth -g off --vo -i /root/your_dump.mcap
```

## 详细说明

- 更完整的运行参数、输入格式、Foxglove、MCAP、VO、GDC、录制与性能说明，请看 [`双目深度stereo_depth使用说明-v20-20260630_163329.pdf`](双目深度stereo_depth使用说明-v20-20260630_163329.pdf)。

