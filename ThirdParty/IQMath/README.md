# IQMath 接入说明

来源：https://github.com/HamsterAPig/IQMath

固定版本：`4c0cd6167a25edf43b5c7ce5cf4cc8b27f3a24fc`（2022-06-11）。

保留上游的 `IQmathLib.h`、`IQmathCPP.h`、`gcc-cm3/IQmathLib-cm3.a`，
以及原始说明 `README.upstream.md`。头文件和二进制库未修改。
使用 ARM GCC 的 Cortex-M3 版本，不使用之前下载的 MSP430/MSP432 库。

## CMake 与 C 代码

顶层工程已链接 `IQMath::IQMath`，应用源码可直接包含：

```c
#include "IQmathLib.h"

_iq24 a = _IQ24(0.25f);
_iq24 b = _IQ24(0.5f);
_iq24 product = _IQ24mpy(a, b);
float value = _IQ24toF(product);
```

优先显式使用 `_iq24` / `_IQ24...` 等带格式的接口。该版本通用接口的默认
配置为 `GLOBAL_Q=20`，与 MSP430 版本的 `GLOBAL_IQ` 宏不同。若使用通用
`_iq` / `_IQ...` 接口，应在包含头文件前统一设置 `GLOBAL_Q`，保持所有调用一致。

Q24 的有符号数范围为 [-128, 128)，不适合直接保存所有 PI 增益、平方结果
或多圈角度。改写电流环时需要先选择缩放比例及各中间量的 Q 格式，并处理溢出。
链接本库不会自动把现有 `float` 运算变成定点运算。

## 当前电流环

`FOC/CurrentControlIQ.c` 使用显式 Q20 运算，默认由 CMake 的
`FOC_USE_IQMATH=ON` 选择。共用参数保存在 `FOC/CurrentControlConfig.h`：

- 环内的 Clarke/Park、低通滤波、PI、矢量限幅和 SVPWM 均使用 Q20。
- 电流环约 10 kHz（20 kHz ADC 采样二分频），周期为 100 微秒；
  电流上限 1 A、电压矢量上限 12 V，沿用原梯形积分规则。
- 预先计算 `Ki*Ts/2`，不将超出 Q20 范围的 Ki 直接转换。
- 两轴限幅后的平方和不超过 288，小于 Q20 正数上限 2048。
- 电流输入的数值保护为 ±16 A，本板 ADC 正常量程约为 ±1.65 A。
- 正余弦仅计算一次，并复用于逆 Park；母线倒数仅在母线值变化时更新。
- 使用 IQmath 平方根计算电压矢量限幅，替代原版的近似平方根。
- 外部的目标电流、ADC 电流和 `current_d/current_q` 仍以浮点数表示，
  仅在边界转换格式；电角度仍由原 AS5600 路径读取并归一化。
- 校准、蜂鸣和电压模式继续使用原 `setPhaseVoltage()` 输出。

Ozone 中继续查看 `current_loop_cycles`：它只记录原始 DWT 周期差值，
包含本次格式转换、环内计算及 PWM 更新，不换算微秒。

切换回原浮点电流环进行对比：

```powershell
cmake --preset Debug -DFOC_USE_IQMATH=OFF
cmake --build --preset Debug
```

恢复 IQmath 版：

```powershell
cmake --preset Debug -DFOC_USE_IQMATH=ON
cmake --build --preset Debug
```

两种构建均保持项目原有的 Debug `-O0` 配置，便于比较。更改算法后需要重新烧录，
实际加速比以目标板 DWT 测量为准。

主机数值检查：`python tests/validate_current_iq.py`。该检查使用实际控制源码，
通过主机 Q20 数学替身和模拟定时器与双精度方程对比；它不验证 ARM 预编译库
在芯片上的执行，也不提供性能测量。

## 上游许可

上游未提供单独的 LICENSE 文件。两个头文件中的 TI Software License Agreement
明确写有：

> Texas Instruments (TI) is supplying this software for use solely and
> exclusively on TI's microcontroller products.

该文字限制使用对象为 TI 微控制器，不能由 GitHub 公开可下载推断其允许用于
STM32。此处保留原文；兼容性验证仅说明工具链能编译链接，不代表许可授予。
