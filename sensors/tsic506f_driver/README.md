# TSic 506F 温度传感器 MicroPython 驱动

## 目录

- [简介](#简介)
- [主要功能](#主要功能)
- [硬件要求](#硬件要求)
- [软件环境](#软件环境)
- [文件结构](#文件结构)
- [文件说明](#文件说明)
- [快速开始](#快速开始)
- [注意事项](#注意事项)
- [版本记录](#版本记录)
- [联系方式](#联系方式)
- [许可协议](#许可协议)

## 简介

本目录提供 TSic 506F 高精度温度传感器的 MicroPython 驱动。驱动基于 RP2040 PIO 实现 `ZACwire` 单线时序解码，读取 11 位温度数据，并使用中值滤波平滑输出，适合需要稳定温度采样、低主循环占用的嵌入式场景。

## 主要功能

1. 基于 RP2040 PIO 双状态机完成脉冲宽度采集和帧起始长脉冲检测；
2. 实现 `ZACwire` 11 位温度数据解码；
3. 支持中值滤波窗口配置；
4. 通过 `micropython.schedule` 将解码移到主上下文，降低中断负载；
5. 提供 `start()`、`stop()`、`deinit()` 完整的资源生命周期管理。

## 硬件要求

| 引脚 | 功能描述 |
|------|----------|
| VCC  | 电源正极（3.3V-5V） |
| GND  | 电源负极 |
| DATA | ZACwire 数据引脚，接 RP2040 GPIO |

推荐测试硬件：

- Raspberry Pi Pico / 其他 RP2040 开发板；
- TSic 506F 温度传感器模块；
- 杜邦线若干。

## 软件环境

| 项目 | 要求 |
|------|------|
| 固件 | MicroPython v1.23+，需包含 `rp2`/PIO 支持 |
| 驱动版本 | v1.0.0 |
| 依赖库 | 无外部依赖 |

## 文件结构

```text
TSic_506F/
├── LICENSE
├── README.md
├── package.json
└── code/
    ├── tsic506.py
    └── main.py
```

## 文件说明

| 文件名 | 功能说明 |
|--------|----------|
| `code/tsic506.py` | TSIC 506F 核心驱动，包含 PIO 程序、解码逻辑和温度读取 |
| `code/main.py` | 测试示例，演示引脚初始化、驱动实例化和周期读取温度 |
| `package.json` | mip/upypi 包配置文件 |
| `LICENSE` | MIT 许可证文件 |

## 快速开始

1. 将 `code/tsic506.py` 和 `code/main.py` 上传到 MicroPython 设备；
2. 按硬件要求连接 TSic 506F 的 VCC、GND 和 DATA 引脚；
3. 运行 `main.py`，或使用以下最小示例：

```python
import time
from machine import Pin
from tsic506 import TSIC506F

time.sleep(3)
data_pin = Pin(6, Pin.OUT, value=0)
time.sleep_ms(10)
data_pin.init(Pin.IN)

sensor = TSIC506F(pin=data_pin, sm=(0, 1), start=True)

while True:
    try:
        print("TSic506F temperature: %.2f C" % sensor.T())
    except Exception as error:
        print("Read error: %s" % str(error))
    time.sleep_ms(500)
```

## 注意事项

| 类别 | 说明 |
|------|------|
| 平台限制 | 驱动依赖 RP2040 PIO，不适用于无 `rp2` 模块的固件 |
| 状态机资源 | 默认占用状态机 0 和 1，避免与项目中的其他 PIO 程序冲突 |
| 首帧等待 | 驱动默认丢弃上电后的第 1 个有效帧（`startup_frames=1`），且首帧解码完成前 `T()` 会抛出 `TSIC506FNotRunning`，需在循环中捕获重试 |
| 引脚配置 | 上电后建议先下拉数据引脚再切换为输入，降低首次启动抖动 |
| 供电 | 确保传感器供电稳定，避免数据线上引入明显噪声 |

## 版本记录

| 版本号 | 日期 | 作者 | 修改说明 |
|--------|------|------|----------|
| v1.0.0 | 2026-09-07 | Robert Hammelrath | 初始版本 |

## 联系方式

如有问题，请通过以下方式联系：

- 作者：Robert Hammelrath
- GitHub：<https://github.com/robert-hh>

## 许可协议

```tex
MIT License

Copyright (c) 2016 Mike Causer

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```
