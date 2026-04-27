#include "zf_common_headfile.h"
#include "define.h"
#include "ins.h"

#define INS_DEG_TO_RAD      (PI / 180.0f)
#define INS_RAD_TO_DEG      (180.0f / PI)

volatile CarPoseState carPose = { 0.0f };

// 将角度规约到 [-pi, pi]
static float wrapAngle(float a)
{
    // 使用循环减少大范围误差
    while (a > PI) a -= 2.0f * PI;
    while (a < -PI) a += 2.0f * PI;
    return a;
}

// 将舵机 PWM duty 计数近似换算为前轮转角，左转为正，右转为负
static float steerPwmToAngle(float steer_pwm)
{
    float pwm = CLIP(steer_pwm, STEER_DUTY_MIN, STEER_DUTY_MAX);

    if (pwm > (float)STEER_DUTY_M) // PWM 在 1887~2400 之间，前轮向左打
    {
        float ratio = (pwm - (float)STEER_DUTY_M) / ((float)STEER_DUTY_L - (float)STEER_DUTY_M);
        return ratio * INS_STEER_LEFT_MAX_RAD;
    }
    else // PWM 在 1375~1887 之间，前轮向右打
    {
        float ratio = ((float)STEER_DUTY_M - pwm) / ((float)STEER_DUTY_M - (float)STEER_DUTY_R);
        return ratio * INS_STEER_RIGHT_MAX_RAD;
    }
}

// 航向角互补修正，使用最短角度差避免跨 ±pi 时跳变
static float blendAngle(float pred, float meas, float gain)
{
    return wrapAngle(pred + gain * wrapAngle(meas - pred));
}

// 初始化车体位姿状态，开机车头方向定义为全局 +X 方向
void carPoseInit(volatile CarPoseState *pt, float yaw_ahrs_deg)
{
    pt->x = 0.0f;
    pt->y = 0.0f;
    pt->yaw = wrapAngle(INS_AHRS_YAW_SIGN * yaw_ahrs_deg * INS_DEG_TO_RAD);
    pt->yaw_deg = pt->yaw * INS_RAD_TO_DEG;

    pt->vx = 0.0f;
    pt->vy = 0.0f;
    pt->speed = 0.0f;
    pt->distance = 0.0f;

    pt->yaw_rate = 0.0f;
    pt->gyro_bias = 0.0f;
    pt->steer_angle = 0.0f;

    pt->initialized = 1;
    pt->position_ready = 0;
}

// 编码器里程计 + 自行车模型 + IMU 航向互补融合
void carPoseUpdate(volatile CarPoseState *pt,
                   float encoder_speed,
                   float steer_pwm,
                   float gyro_z_deg_s,
                   float yaw_ahrs_deg,
                   float dt)
{
    float yaw_ahrs = wrapAngle(INS_AHRS_YAW_SIGN * yaw_ahrs_deg * INS_DEG_TO_RAD);

    if (!pt->initialized)
    {
        carPoseInit(pt, yaw_ahrs_deg);
    }

    if (dt <= 0.0f)
    {
        pt->yaw = blendAngle(pt->yaw, yaw_ahrs, INS_AHRS_YAW_GAIN);
        pt->yaw_deg = pt->yaw * INS_RAD_TO_DEG;
        return;
    }

    // 限制异常时间间隔，避免 IMU 时间戳跳变造成一次性大积分
    if (dt > INS_DT_MAX)
    {
        dt = INS_DT_MAX;
    }

    // 编码器速度低通滤波，编码器方向如与坐标系相反可调整 INS_ENCODER_SPEED_SIGN
    float speed_meas = INS_ENCODER_SPEED_SIGN * encoder_speed;
    pt->speed += INS_SPEED_LPF_ALPHA * (speed_meas - pt->speed);
    pt->steer_angle = steerPwmToAngle(steer_pwm);

    // 陀螺仪单位由 deg/s 转换为 rad/s，并扣除静止时估计出的零偏
    float gyro_raw = INS_GYRO_Z_SIGN * gyro_z_deg_s * INS_DEG_TO_RAD;
    float gyro_unbiased = gyro_raw - pt->gyro_bias;
    float abs_speed = fabsf(pt->speed);
    float abs_gyro = fabsf(gyro_unbiased);

    // 静止检测：速度和角速度都很小时，冻结速度并慢速更新 Z 轴陀螺零偏
    if (abs_speed < INS_ZERO_SPEED_MPS && abs_gyro < INS_ZERO_GYRO_RADPS)
    {
        pt->gyro_bias += INS_GYRO_BIAS_ALPHA * (gyro_raw - pt->gyro_bias);
        pt->speed = 0.0f;
        abs_speed = 0.0f;
        gyro_unbiased = 0.0f;
    }

    // 自行车模型估计航向角速度：yaw_rate = v / L * tan(delta)
    float yaw_rate_model = 0.0f;
    if (INS_WHEEL_BASE_M > 1e-4f)
    {
        yaw_rate_model = pt->speed / INS_WHEEL_BASE_M * tanf(pt->steer_angle);
    }

    // 短期主要相信陀螺，车辆模型用于约束低价 IMU 的漂移和噪声
    pt->yaw_rate = INS_GYRO_MODEL_WEIGHT * gyro_unbiased
                 + (1.0f - INS_GYRO_MODEL_WEIGHT) * yaw_rate_model;

    // AHRS yaw 只做慢修正，避免其瞬时漂移直接拉动位姿
    float yaw_pred = wrapAngle(pt->yaw + pt->yaw_rate * dt);
    pt->yaw = blendAngle(yaw_pred, yaw_ahrs, INS_AHRS_YAW_GAIN);
    pt->yaw_deg = pt->yaw * INS_RAD_TO_DEG;

    // 坐标系：开机车头方向为全局 +X，车辆左侧为全局 +Y
    float vx = pt->speed * cosf(pt->yaw);
    float vy = pt->speed * sinf(pt->yaw);

    // 位置使用速度梯形积分，第一帧无上一速度时退化为欧拉积分
    if (!pt->position_ready)
    {
        pt->x += vx * dt;
        pt->y += vy * dt;
        pt->position_ready = 1;
    }
    else
    {
        pt->x += 0.5f * (pt->vx + vx) * dt;
        pt->y += 0.5f * (pt->vy + vy) * dt;
    }

    pt->distance += abs_speed * dt;
    pt->vx = vx;
    pt->vy = vy;
}
