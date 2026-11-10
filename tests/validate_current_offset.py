"""用主机 HAL 替身执行 main.c 中实际的零点校准函数和 ADC 回调。

运行：python tests/validate_current_offset.py
验证分通道均值、电流换算、采样超时、异常零点和中断重新使能。
不验证硬件模拟电路或实际 ADC 触发时序。
"""
from pathlib import Path
import shutil
import subprocess


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "build/current_offset_validation"
OUT.mkdir(parents=True, exist_ok=True)
source = (ROOT / "Core/Src/main.c").read_text(encoding="utf-8")


def section(name):
    return source.split(f"/* USER CODE BEGIN {name} */", 1)[1].split(
        f"/* USER CODE END {name} */", 1)[0]


def function(signature):
    start = source.index(signature)
    brace = source.index("{", start)
    depth = 1
    end = brace + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


stub = r"""
#include <stdint.h>
#include <assert.h>
#include <math.h>
#include <stdio.h>
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef struct { void *Instance; } ADC_HandleTypeDef;
#define ADC1 ((void *)1)
#define DRV_EN_GPIO_Port ((void *)2)
#define DRV_EN_Pin 4
#define GPIO_PIN_RESET 0
#define ADC_INJECTED_RANK_1 1
#define ADC_INJECTED_RANK_2 2
#define ADC_FLAG_JEOC 1
#define ADC_IT_JEOC 1
#define CURRENT_ADC_DECIMATION 2U
static uint32_t tick, delivered, rearmed, cleared, updates;
static uint16_t raw_a, raw_b, base_a = 2000, base_b = 2100;
static int deliver_samples = 1, driver = 1, control_enabled = 1;
static float electrical_angle;
static ADC_HandleTypeDef adc = {ADC1};
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc);
static void HAL_GPIO_WritePin(void *port, int pin, int value) {
    (void)port; (void)pin; driver = value;
}
static void CurrentControl_Enable(uint8_t enabled) { control_enabled = enabled; }
static void CurrentControl_Update(float a, float b, float angle) {
    (void)a; (void)b; (void)angle; updates++;
}
static uint32_t HAL_ADCEx_InjectedGetValue(ADC_HandleTypeDef *adc, int rank) {
    (void)adc; return rank == 1 ? raw_a : raw_b;
}
#define __HAL_ADC_CLEAR_FLAG(adc, flag) ((void)(adc), (void)(flag), cleared++)
#define __HAL_ADC_ENABLE_IT(adc, flag) ((void)(adc), (void)(flag), rearmed++)
static void sample(void) {
    raw_a = base_a + (delivered & 1U);
    raw_b = base_b + (delivered & 1U);
    delivered++;
    HAL_ADCEx_InjectedConvCpltCallback(&adc);
}
static uint32_t HAL_GetTick(void) {
    if (deliver_samples) for (unsigned i = 0; i < 20; ++i) sample();
    return tick++;
}
static void HAL_Delay(uint32_t delay) {
    assert(driver == 0 && control_enabled == 0);
    for (uint32_t i = 0; i < delay; ++i) {
        if (deliver_samples) sample();
        tick++;
    }
}
"""

checks = r"""
int main(void) {
    /* 开始校准前的采样不能调用控制器，但必须重新使能 JEOC。 */
    sample();
    assert(updates == 0 && rearmed == 1 && cleared == 1);
    assert(CurrentSense_CalibrateOffset() == HAL_OK);
    assert(driver == 0 && control_enabled == 0);
    assert(current_offset_status == 2 && current_offset_sample_count == 1024);
    assert(current_offset_adc_a == 2000.5f && current_offset_adc_b == 2100.5f);
    assert(updates == 0 && cleared == rearmed && rearmed == delivered);

    raw_a = 2000; raw_b = 2101;
    HAL_ADCEx_InjectedConvCpltCallback(&adc);
    assert(updates == 0);
    HAL_ADCEx_InjectedConvCpltCallback(&adc);
    assert(updates == 1);
    assert(fabsf(phase_current_a + 0.5f * 3.3f / 4096) < 1e-7f);
    assert(fabsf(phase_current_b - 0.5f * 3.3f / 4096) < 1e-7f);
    assert(fabsf(phase_current_c) < 1e-7f);
    assert(current_offset_sample_count == 1024);

    ADC_HandleTypeDef other = {(void *)3};
    uint32_t before = rearmed;
    HAL_ADCEx_InjectedConvCpltCallback(&other);
    assert(rearmed == before);

    base_a = 0;
    assert(CurrentSense_CalibrateOffset() == HAL_ERROR);
    assert(current_offset_status == 3 && driver == 0 && control_enabled == 0);
    before = updates;
    sample(); sample();
    assert(updates == before && rearmed == cleared);
    base_a = 2000; base_b = 4094;
    assert(CurrentSense_CalibrateOffset() == HAL_ERROR);
    assert(current_offset_status == 3 && driver == 0);

    /* 停止触发以及毫秒计数回绕时，都应超时退出。 */
    deliver_samples = 0;
    tick = UINT32_MAX - 30;
    assert(CurrentSense_CalibrateOffset() == HAL_TIMEOUT);
    assert(current_offset_status == 3 && current_offset_sample_count == 0);
    assert(driver == 0 && control_enabled == 0);
    base_b = 2100; deliver_samples = 1;
    assert(CurrentSense_CalibrateOffset() == HAL_OK);
    assert(current_offset_sample_count == 1024);
    puts("PASS: separate offsets, conversion/decimation, invalid offsets, timeout/wrap, JEOC rearm");
    return 0;
}
"""

test_source = OUT / "current_offset_test.c"
test_source.write_text(
    stub + section("PD") + section("PV")
    + function("static HAL_StatusTypeDef CurrentSense_CalibrateOffset(void)")
    + function("void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)")
    + checks, encoding="utf-8")
compiler = shutil.which("gcc") or r"C:\TDM-GCC-64\bin\gcc.exe"
executable = OUT / "current_offset_test.exe"
subprocess.run([compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
                str(test_source), "-o", str(executable)], check=True)
subprocess.run([str(executable)], check=True)
