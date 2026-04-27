#ifndef CODE_INS_H_
#define CODE_INS_H_

/*
 * 编码器里程计 + 航向角互补融合。
 *
 * 坐标系约定：
 *   - 原点：小车上电完成 IMU 置零时的位置
 *   - +X：小车开机时车头朝向
 *   - +Y：小车开机时车体左侧方向
 *   - yaw：弧度制，逆时针为正，范围 [-pi, pi]
 *
 * 下列参数均需要根据实车标定。
 */
#ifndef INS_WHEEL_BASE_M
#define INS_WHEEL_BASE_M           (0.286f)      // 前后轴距，单位 m，实测 28.6 cm = 0.286m
#endif

#ifndef INS_STEER_LEFT_MAX_RAD
#define INS_STEER_LEFT_MAX_RAD     (0.4277287f)  // 左满舵对应前轮转角，约 +24.507047 deg
#endif

#ifndef INS_STEER_RIGHT_MAX_RAD
#define INS_STEER_RIGHT_MAX_RAD    (-0.3505551f) // 右满舵对应前轮转角，约 -20.085329 deg
#endif

/* 如果实车传感器方向与坐标系相反，优先修改下面三个符号，不要改公式。 */
#ifndef INS_ENCODER_SPEED_SIGN
#define INS_ENCODER_SPEED_SIGN     (-1.0f)       // 编码器前进方向符号
#endif

#ifndef INS_GYRO_Z_SIGN
#define INS_GYRO_Z_SIGN            (1.0f)        // IMU Z 轴角速度符号
#endif

#ifndef INS_AHRS_YAW_SIGN
#define INS_AHRS_YAW_SIGN          (1.0f)        // AHRS yaw 角符号
#endif

#ifndef INS_SPEED_LPF_ALPHA
#define INS_SPEED_LPF_ALPHA        (0.35f)       // 编码器速度一阶低通滤波系数
#endif

#ifndef INS_GYRO_MODEL_WEIGHT
#define INS_GYRO_MODEL_WEIGHT      (0.70f)       // 航向角速度中陀螺仪所占权重
#endif

#ifndef INS_AHRS_YAW_GAIN
#define INS_AHRS_YAW_GAIN          (0.06f)       // AHRS yaw 对融合航向角的慢修正增益
#endif

#ifndef INS_GYRO_BIAS_ALPHA
#define INS_GYRO_BIAS_ALPHA        (0.002f)      // 静止时 Z 轴陀螺零偏更新系数
#endif

#ifndef INS_ZERO_SPEED_MPS
#define INS_ZERO_SPEED_MPS         (0.02f)       // 静止检测速度阈值，单位 m/s
#endif

#ifndef INS_ZERO_GYRO_RADPS
#define INS_ZERO_GYRO_RADPS        (0.0174533f)  // 静止检测角速度阈值，约 1 deg/s
#endif

#ifndef INS_DT_MAX
#define INS_DT_MAX                 (0.100f)      // 最大积分时间间隔，单位 s
#endif

typedef struct {
    float x;               // 全局 X 坐标，单位 m
    float y;               // 全局 Y 坐标，单位 m
    float yaw;             // 融合航向角，单位 rad，范围 [-pi, pi]
    float yaw_deg;         // 融合航向角，单位 deg，便于显示和上行发送

    float vx;              // 全局 X 方向速度，单位 m/s
    float vy;              // 全局 Y 方向速度，单位 m/s
    float speed;           // 滤波后的车体前向速度，单位 m/s
    float distance;        // 累计行驶距离，单位 m

    float yaw_rate;        // 融合航向角速度，单位 rad/s
    float gyro_bias;       // Z 轴陀螺零偏，单位 rad/s
    float steer_angle;     // 由舵机 PWM 估算的前轮转角，单位 rad

    unsigned char initialized;
    unsigned char position_ready;
} CarPoseState;

extern volatile CarPoseState carPose;

void carPoseInit(volatile CarPoseState *pt, float yaw_ahrs_deg);
void carPoseUpdate(volatile CarPoseState *pt,
                   float encoder_speed,
                   float steer_pwm,
                   float gyro_z_deg_s,
                   float yaw_ahrs_deg,
                   float dt);

#endif /* CODE_INS_H_ */
