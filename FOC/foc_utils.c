#include "foc_utils.h"
#include <stdint.h>

/***************************************************************************/
// 含 65 个元素的正弦查找表，使用线性插值（来自 SimpleFOC 2.4）
// 正弦值精度为 16 位，插值小数部分为 8 位，
// 查表索引为 6 位；在 [0, 2PI) 内相对标准库正弦函数的均方根误差为 0.00006480。
// 输入角度必须位于 [0, 2PI)。
float _sin(float a){
  static const uint16_t sine_array[65] = {0,804,1608,2411,3212,4011,4808,5602,6393,7180,7962,8740,9512,10279,11039,11793,12540,13279,14010,14733,15447,16151,16846,17531,18205,18868,19520,20160,20788,21403,22006,22595,23170,23732,24279,24812,25330,25833,26320,26791,27246,27684,28106,28511,28899,29269,29622,29957,30274,30572,30853,31114,31357,31581,31786,31972,32138,32286,32413,32522,32610,32679,32729,32758,32768};
  int32_t t1, t2;
  unsigned int i = (unsigned int)(a * (64*4*256.0f/_2PI));
  int frac = i & 0xff;
  i = (i >> 8) & 0xff;
  if (i < 64) {
    t1 = (int32_t)sine_array[i]; t2 = (int32_t)sine_array[i+1];
  }
  else if(i < 128) {
    t1 = (int32_t)sine_array[128 - i]; t2 = (int32_t)sine_array[127 - i];
  }
  else if(i < 192) {
    t1 = -(int32_t)sine_array[-128 + i]; t2 = -(int32_t)sine_array[-127 + i];
  }
  else {
    t1 = -(int32_t)sine_array[256 - i]; t2 = -(int32_t)sine_array[255 - i];
  }
  return (1.0f/32768.0f) * (t1 + (((t2 - t1) * frac) >> 8));
}
/***************************************************************************/
// 利用固定大小的查找表近似计算余弦值
// 精度为 ±0.005
// 输入角度必须位于 0 到 2PI 之间
float _cos(float a){
  float a_sin = a + _PI_2;
  a_sin = a_sin > _2PI ? a_sin - _2PI : a_sin;
  return _sin(a_sin);
}
/***************************************************************************/
// 一次调用同时计算角度的正弦值和余弦值
void _sincos(float a, float* s, float* c){
  *s = _sin(a);
  *c = _cos(a);
}
/***************************************************************************/
// 将弧度角归一化到 [0, 2PI)
float _normalizeAngle(float angle){
  float a = fmod(angle, _2PI);
  return a >= 0 ? a : (a + _2PI);
}
/***************************************************************************/
// 计算电角度
float _electricalAngle(float shaft_angle, int pole_pairs) {
  return (shaft_angle * pole_pairs);
}
/***************************************************************************/
// 平方根近似计算，算法参考：
// https://reprap.org/forum/read.php?147,219210
// https://en.wikipedia.org/wiki/Fast_inverse_square_root
float _sqrtApprox(float number) {//轻量级近似计算
  union {
    float    f;
    uint32_t i;
  } y = { .f = number };
  y.i = 0x5f375a86 - ( y.i >> 1 );
  return number * y.f;
}
/***************************************************************************/
