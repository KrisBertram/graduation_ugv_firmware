#include "zf_common_headfile.h"
#include "define.h"
#include "mmath.h"

// 通用线性映射：将 value 从 [in_min, in_max] 映射到 [out_min, out_max]
int mapLinear(int value, int in_min, int in_max, int out_min, int out_max)
{
    if (value <= in_min) return out_min;
    if (value >= in_max) return out_max;

    return (int)roundf((float)(value - in_min) * (float)(out_max - out_min) / (float)(in_max - in_min) + (float)out_min);
}

// 三段式线性映射：输入区间分成 min -> mid 和 mid -> max 两段
int mapTri(int value,
           int in_min, int in_mid, int in_max,
           int out_min, int out_mid, int out_max)
{
    if (value <= in_min) return out_min;
    if (value >= in_max) return out_max;

    // 左半区：in_min → in_mid
    if (value <= in_mid)  return (int)roundf((float)(value - in_min) * (float)(out_mid - out_min) / (float)(in_mid - in_min) + (float)out_min);

    // 右半区：in_mid → in_max
    return (int)roundf((float)(value - in_mid) * (float)(out_max - out_mid) / (float)(in_max - in_mid) + (float)out_mid);
}
