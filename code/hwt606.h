#ifndef CODE_HWT606_H_
#define CODE_HWT606_H_
#include "mmath.h"

#define UPDATE_ACC      BIT(0) // 加速度数据更新 (AX, AY, AZ)
#define UPDATE_GYRO     BIT(1) // 角速度数据更新 (GX, GY, GZ)
#define UPDATE_MAG      BIT(2) // 磁力计数据更新 (MX, MY, MZ)
#define UPDATE_ANGLE    BIT(3) // 欧拉角数据更新 (Roll, Pitch, Yaw)
#define UPDATE_QUAT     BIT(4) // 四元数数据更新 (Q0, Q1, Q2, Q3)
#define UPDATE_TIME     BIT(5) // 片上时间更新
#define UPDATE_TEMP     BIT(6) // 温度
#define UPDATE_READ     BIT(7) // 其它数据

typedef struct {
    float gx, gy, gz;
    float ax, ay, az;
    float Q0, Q1, Q2, Q3;
    float roll, pitch, yaw;
    uint16 yy, mm, dd, hh, mn, ss, ms;
    float timestamp, timestampLast;
} IMU_t;

extern uint8 hwtInitialized;
extern volatile uint8 hwtUpdate;

extern IMU_t imu_dat;

void WitSerialWrite(uint8_t *data, uint32_t len);
void WitDelayms(uint16_t ucMs);
void WitRegUpdateHandler(uint32_t uiReg, uint32_t uiRegNum);
float imuGetTimeStamp(const IMU_t* imu);

void imuCalibrateAcc(void);
void imuZeroXY(void);
void imuZeroZ(void);

#endif /* CODE_HWT606_H_ */
