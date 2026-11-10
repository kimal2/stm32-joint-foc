"""在主机上将实际 Q20 控制代码与双精度方程对比。

运行：python tests/validate_current_iq.py
需要主机 GCC；临时产物写入 build/iqmath_validation/native。
主机替身提供 Q20 数学和定时器寄存器，不验证 ARM 二进制库的实现或耗时。
"""
import ctypes
import math
import random
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "build/iqmath_validation/native"
OUT.mkdir(parents=True, exist_ok=True)

(OUT / "IQmathLib.h").write_text(r"""
#include <stdint.h>
#include <math.h>
#include <assert.h>
typedef int32_t _iq20;
#define _IQ20(x) ((_iq20)((x) * 1048576.0f))
static inline _iq20 _IQ20mpy(_iq20 a, _iq20 b) {
    int64_t value = ((int64_t)a * b) >> 20;
    assert(value >= INT32_MIN && value <= INT32_MAX);
    return (_iq20)value;
}
static inline _iq20 _IQ20div(_iq20 a, _iq20 b) {
    assert(b != 0);
    int64_t value = (int64_t)a * 1048576 / b;
    assert(value >= INT32_MIN && value <= INT32_MAX);
    return (_iq20)value;
}
static inline _iq20 _IQ20sin(_iq20 a) {
    return (_iq20)(sin((double)a / 1048576) * 1048576);
}
static inline _iq20 _IQ20cos(_iq20 a) {
    return (_iq20)(cos((double)a / 1048576) * 1048576);
}
static inline _iq20 _IQ20sqrt(_iq20 a) {
    assert(a >= 0);
    return (_iq20)(sqrt((double)a / 1048576) * 1048576);
}
static inline float _IQ20toF(_iq20 a) { return (float)a / 1048576.0f; }
""", encoding="ascii")

(OUT / "stm32f1xx.h").write_text(r"""
#include <stdint.h>
typedef struct { uint32_t CTRL, CYCCNT; } MockDWT;
typedef struct { uint32_t DEMCR; } MockCoreDebug;
extern MockDWT mock_dwt;
extern MockCoreDebug mock_debug;
#define DWT (&mock_dwt)
#define CoreDebug (&mock_debug)
#define CoreDebug_DEMCR_TRCENA_Msk 1U
#define DWT_CTRL_CYCCNTENA_Msk 1U
#define __DSB() ((void)0)
#define __ISB() ((void)0)
""", encoding="ascii")

(OUT / "tim.h").write_text(r"""
#include <stdint.h>
typedef struct { uint32_t arr, ccr[3]; } MockTimer;
extern MockTimer htim2;
#define TIM_CHANNEL_1 0
#define TIM_CHANNEL_2 1
#define TIM_CHANNEL_3 2
#define __HAL_TIM_GET_AUTORELOAD(t) ((t)->arr)
#define __HAL_TIM_SET_COMPARE(t, ch, value) ((t)->ccr[ch] = (value))
""", encoding="ascii")

(OUT / "driver.c").write_text(r"""
#include "CurrentControlIQ.c"
MockDWT mock_dwt;
MockCoreDebug mock_debug;
MockTimer htim2 = {1799, {0, 0, 0}};
float voltage_power_supply = 24.0f;
volatile uint8_t beepPlaying;
uint32_t test_ccr(unsigned channel) { return htim2.ccr[channel]; }
void test_beep(unsigned value) { beepPlaying = value; }
void test_bus(float value) { voltage_power_supply = value; }
float test_integral(unsigned axis) {
    return _IQ20toF(axis ? pi_q.integral : pi_d.integral);
}
""", encoding="ascii")

dll_path = OUT / "current_iq.dll"
subprocess.run([
    "gcc", "-std=c11", "-O2", "-ftrapv", "-shared", "-I", str(OUT),
    "-I", str(ROOT / "FOC"), str(OUT / "driver.c"), "-lm", "-o", str(dll_path),
], check=True)
lib = ctypes.CDLL(str(dll_path))
lib.CurrentControl_SetTarget.argtypes = [ctypes.c_float]
lib.CurrentControl_Update.argtypes = [ctypes.c_float] * 3
lib.CurrentControl_Enable.argtypes = [ctypes.c_uint8]
lib.test_ccr.argtypes = [ctypes.c_uint]
lib.test_ccr.restype = ctypes.c_uint32
lib.test_integral.argtypes = [ctypes.c_uint]
lib.test_integral.restype = ctypes.c_float
lib.test_bus.argtypes = [ctypes.c_float]
current_d = ctypes.c_float.in_dll(lib, "current_d")
current_q = ctypes.c_float.in_dll(lib, "current_q")


def clamp(value, limit):
    return max(-limit, min(limit, value))


def ccr():
    return tuple(lib.test_ccr(i) for i in range(3))


class Reference:
    def __init__(self):
        self.filtered = [0.0, 0.0]
        self.integral = [0.0, 0.0]
        self.error_prev = [0.0, 0.0]

    def step(self, ia, ib, theta, target, bus=24):
        alpha = clamp(ia, 16)
        beta = (alpha + 2 * clamp(ib, 16)) / math.sqrt(3)
        s, c = math.sin(theta), math.cos(theta)
        measured = [alpha * c + beta * s, -alpha * s + beta * c]
        for axis in range(2):
            self.filtered[axis] += .0001 / (.0001592 + .0001) * (
                measured[axis] - self.filtered[axis])
        errors = [-self.filtered[0], clamp(target, 1) - self.filtered[1]]
        voltage = []
        for axis, error in enumerate(errors):
            self.integral[axis] = clamp(self.integral[axis] +
                10.32 * 2 * math.pi * 200 * .0001 / 2 * (
                    error + self.error_prev[axis]), 12)
            self.error_prev[axis] = error
            voltage.append(clamp(.00326 * 2 * math.pi * 200 * error +
                                 self.integral[axis], 12))
        ud, uq = voltage
        magnitude = math.hypot(ud, uq)
        if magnitude > 12:
            ud, uq = ud * 12 / magnitude, uq * 12 / magnitude
        alpha, beta = c * ud - s * uq, s * ud + c * uq
        phases = [alpha, -alpha / 2 + math.sqrt(3) / 2 * beta,
                  -alpha / 2 - math.sqrt(3) / 2 * beta]
        center = bus / 2 - (max(phases) + min(phases)) / 2
        return tuple(int(max(0, min(1, (u + center) / bus)) * 1800)
                     for u in phases)


lib.CurrentControl_Init()
lib.CurrentControl_Enable(1)
ref = Reference()
rng = random.Random(4310)
max_current_error = 0
max_ccr_error = 0
max_integral_error = 0
for n in range(20000):
    # 同时覆盖实际 ADC 范围、正反转矩、阶跃、随机输入及矢量限幅。
    ia, ib = [ctypes.c_float(rng.uniform(-1.65, 1.65)).value for _ in range(2)]
    theta = ctypes.c_float((n * .071) % (2 * math.pi)).value
    target = ctypes.c_float(1.5 * math.sin(n * .019)).value
    lib.CurrentControl_SetTarget(target)
    lib.CurrentControl_Update(ia, ib, theta)
    expected = ref.step(ia, ib, theta, target)
    max_current_error = max(max_current_error,
        abs(current_d.value - ref.filtered[0]), abs(current_q.value - ref.filtered[1]))
    max_integral_error = max(max_integral_error, *(
        abs(lib.test_integral(i) - ref.integral[i]) for i in range(2)))
    max_ccr_error = max(max_ccr_error, *(abs(a - b) for a, b in zip(ccr(), expected)))
    assert all(0 <= value <= 1800 for value in ccr())
assert max_current_error < .00005, max_current_error
assert max_integral_error < .005, max_integral_error
assert max_ccr_error <= 3, max_ccr_error

# 暂停时不可覆盖 PWM；禁用后重新使能须清除滤波和积分状态。
before = ccr()
lib.test_beep(1)
lib.CurrentControl_Update(1, 1, 0)
assert ccr() == before
lib.test_beep(0)
lib.CurrentControl_Enable(0)
lib.CurrentControl_Update(1, 1, 0)
assert ccr() == before
lib.CurrentControl_SetTarget(0)
lib.CurrentControl_Enable(1)
lib.CurrentControl_Update(0, 0, 0)
assert all(abs(value - 900) <= 1 for value in ccr())
assert current_d.value == current_q.value == 0

# 极端输入及不同母线值不得导致环内定点溢出或 CCR 越界。
for bus in (1, 12, 24, 48, 512):
    lib.test_bus(bus)
    for ia in (-1e6, -16, 0, 16, 1e6):
        for ib in (-1e6, -16, 0, 16, 1e6):
            for theta in (0, math.pi / 2, math.pi, 2 * math.pi - 1e-6):
                lib.CurrentControl_Update(ia, ib, theta)
                assert all(0 <= value <= 1800 for value in ccr()), ccr()
for bus in (0, -24, float("nan"), float("inf")):
    lib.test_bus(bus)
    lib.CurrentControl_Update(1, 1, 0)
    assert ccr() == (0, 0, 0)

print(f"PASS: 20000 waveform samples, max current error={max_current_error:.8f} A, "
      f"max integral error={max_integral_error:.8f} V, max PWM error={max_ccr_error} counts")
print("PASS: vector limiting, pause/reset, input bounds and bus voltage bounds")
