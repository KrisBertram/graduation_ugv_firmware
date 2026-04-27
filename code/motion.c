#include "zf_common_headfile.h"
#include "define.h"
#include "motion.h"

#define EN_WHEEL_DIAMETER   (0.082f)        // 车轮直径
#define EN_REDUCTION_RATIO  (11.36f)        // 电机减速比
#define EN_LINE_COUNT       (1024.0f)       // 编码器线数
#define EN_DT               (0.005f)        // 采样周期（秒）
#define EN_TRANS_FACTOR     (0.0044291f)    // 单位转换系数，编码器脉冲数 -> 实际车速 (m/s)

int16 encoder_pulse = 0;
float encoder_v = 0.0f;
float steer_duty = (float)STEER_DUTY_M;
float motor_duty = 0.0f;

float encoderSpeedCalculate(int16 pulse)
{
#if defined(EN_TRANS_FACTOR)
    return (float)pulse * EN_TRANS_FACTOR;
#else
    return (float)pulse * (PI * EN_WHEEL_DIAMETER) / (EN_LINE_COUNT * EN_REDUCTION_RATIO * EN_DT);
#endif
}
