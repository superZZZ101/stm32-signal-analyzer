# STM32 信号测量与频谱分析仪（2026 电赛）

基于 **STM32F407** 的信号分析固件：以 4.212 MSPS 采集信号，用 CMSIS-DSP 做 FFT，
完成波形显示、参数测量、频谱绘制和多分量分解，并通过 USART HMI 串口屏显示结果。

本仓库为 2026 年全国大学生电子设计竞赛（NUEDC）赛题作品的开源整理版本。

## 功能

| 任务 | 入口函数 | 说明 |
| --- | --- | --- |
| 波形显示 | `Measurement_Task1_PrepareWaveform` | 截取 1 个或 3 个完整周期（600 点）送屏显示 |
| 参数测量 | `Measurement_Task2_MeasureParameters` | 峰峰值 Vpp、真有效值 RMS、基波频率 |
| 频谱绘制 | `Measurement_Task3_PrepareSpectrum` | 0–500 kHz 正频率频谱，600 根柱状图 |
| 分量测量 | `Measurement_Task4_MeasureComponents` | 最多 3 个频率分量的频率与正弦峰值幅度 |
| 混叠判别 | `Measurement_CompareRawLowBandSignatures` | 双采样率互比对，剔除采样混叠伪谱线 |

### 双采样率混叠判别

前端采样率在 **4.212 MSPS（正常）** 与 **3.0086 MSPS（探针）** 两档间切换。
真实信号分量在两个采样率下频率不变，而折叠进来的混叠分量会随采样率移动；
比对两条低频段特征谱线即可判定并剔除混叠，再用稳定分量重建波形、参数与频谱
（`Measurement_Recover*`）。两档采样率的实测值经过 10 kHz / 100 kHz / 500 kHz
基准校准，已计入 HSI 时钟约 0.29% 的正偏差。

### DSP 参数

- 采集 16384 点原始样本，低通滤波后 4:1 抽取到 4096 点做 FFT
- 频率分辨率 ≈ 257 Hz（4.212 MSPS / 4 / 4096），满足 ≤500 Hz 要求
- 前端模拟增益 `MEASUREMENT_FRONT_END_GAIN`（标称 4.951），所有结果折算到 BNC 输入端

## 硬件连接

| 信号 | 引脚 |
| --- | --- |
| 被测信号 → 运放前端 → ADC | PC2 |
| USART HMI 串口屏 | USART1：PA9 (TX)、PA10 (RX)，9600 8N1 |

屏幕工程（800×480）包含 `page0` 介绍页、`page1` 时域波形与参数页、`page2` 频谱页。
600 点波形采用 `addt` 透明波形的握手协议（`FE`/`FD` 确认后才发原始字节），
避免数据被串口屏当作指令解析。协议细节见 [HMI_SCREEN_SETUP.md](HMI_SCREEN_SETUP.md)。

## 构建

工程由 STM32CubeMX 生成，使用 CMake + Ninja，工具链为 `arm-none-eabi-gcc`：

```bash
cmake --preset Debug        # 或 Release
cmake --build --preset Debug
```

产物为 `build/<Config>/try.elf`，可用 ST-LINK 烧录：

```bash
st-flash write build/Debug/try.elf 0x8000000
```

第三方组件：STM32F4 HAL（按 ADC 模块裁剪）、CMSIS-DSP（预编译
`libarm_cortexM4lf_math.a`）。

## 代码结构

```text
Core/Src/adc_capture.c         DMA 双缓冲采集与统计
Core/Src/measurement_tasks.c   测量、FFT、混叠判别与重建
Core/Src/tjc_usart_hmi.c       串口屏协议与波形下发
Core/Src/main.c                主循环与任务调度
tools/analyze_adc_snapshot.py  ADC 快照离线分析脚本
third_party/                    HAL / CMSIS-DSP
```

## 备注

- `build/` 为本地编译产物，已在 `.gitignore` 中忽略。
- 屏幕工程文件（`.HMI` / `.tft`）需用淘晶驰 USART HMI 上位机打开，未包含在本仓库中。
