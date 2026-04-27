#include "zf_common_headfile.h"
#include "wit_c_sdk.h"
#include "define.h"
#include "hwt606.h"

uint8 hwtInitialized = FALSE;
volatile uint8 hwtUpdate = 0;

IMU_t imu_dat = { 0.0f };

/**
 * @brief 串口数据发送函数（用于注册到 WIT SDK）
 * @param data 待发送数据的缓冲区指针
 * @param len  待发送数据的长度（字节数）
 */
void WitSerialWrite(uint8_t *data, uint32_t len) { uart_write_buffer(HWT606_UART, data, len); }

/**
 * @brief 毫秒延时函数（用于注册到 WIT SDK）
 * @param ucMs  需要延时的毫秒数
 */
void WitDelayms(uint16_t ucMs) { system_delay_ms((uint32)ucMs); }

/**
 * @brief 传感器数据更新回调函数
 *
 * 当传感器数据被解析并更新到寄存器后，SDK 会调用此函数
 * 应用程序可以在此获取最新的传感器数据并进行处理
 *
 * @param uiReg     更新的起始寄存器地址
 * @param uiRegNum  更新的寄存器数量
 */
void WitRegUpdateHandler(uint32_t uiReg, uint32_t uiRegNum)
{
    for (uint32_t i = 0, ie = uiRegNum; i < ie; ++i)
    {
        switch(uiReg)
        {
            case AZ:    // 加速度
                BIT_SET_TRUE(hwtUpdate, UPDATE_ACC);
                break;

            case GZ:    // 角速度
                BIT_SET_TRUE(hwtUpdate, UPDATE_GYRO);
                break;

            case HZ:    // 磁力计
                BIT_SET_TRUE(hwtUpdate, UPDATE_MAG);
                break;

            case Yaw:   // 欧拉角
                BIT_SET_TRUE(hwtUpdate, UPDATE_ANGLE);
                break;

            case MS:    // 片上时间
                BIT_SET_TRUE(hwtUpdate, UPDATE_TIME);
                break;

            case q3:    // 四元数
                BIT_SET_TRUE(hwtUpdate, UPDATE_QUAT);
                break;

            case TEMP:  // 温度
                BIT_SET_TRUE(hwtUpdate, UPDATE_TEMP);
                break;

            default:    // 其他数据
                BIT_SET_TRUE(hwtUpdate, UPDATE_READ);
                break;
        }

        ++uiReg;
    }
}

float imuGetTimeStamp(const IMU_t* imu)
{
    float t = 0.0f;
    t += (float)imu->hh * 3600.0f;
    t += (float)imu->mn * 60.0f;
    t += (float)imu->ss;
    t += (float)imu->ms * 0.001f;
    return t;
}

// ★ 发送一帧 5 字节的 IMU 指令
static inline void imuSend5(uint8 b1, uint8 b2, uint8 b3, uint8 b4, uint8 b5)
{
    uint8 buf[5] = {b1, b2, b3, b4, b5};
    uart_write_buffer(HWT606_UART, buf, 5);
}

// ★ 解锁，指令：FF AA 69 88 B5
static inline void imuUnlock(void)
{
    imuSend5(0xFF, 0xAA, 0x69, 0x88, 0xB5);
}

// ★ 保存指令，指令：FF AA 00 00 00
static inline void imuSave(void)
{
    imuSend5(0xFF, 0xAA, 0x00, 0x00, 0x00);
}

// ★ 加速度校准
void imuCalibrateAcc(void)
{
    // 1. 解锁，延时 200ms
    imuUnlock();
    system_delay_ms(200);

    // 2. 开始校准，等待 4 秒：FF AA 01 01 00
    imuSend5(0xFF, 0xAA, 0x01, 0x01, 0x00);
    system_delay_ms(4000);

    // 3. 退出校准，延时 100ms：FF AA 01 00 00
    imuSend5(0xFF, 0xAA, 0x01, 0x00, 0x00);
    system_delay_ms(100);

    // 4. 保存设置
    imuSave();
}

// ★ 角度参考（XY 归零）
void imuZeroXY(void)
{
    // 1. 解锁，延时 200ms
    imuUnlock();
    system_delay_ms(200);

    // 2. XY 角度置零，等待 3 秒：FF AA 01 08 00
    imuSend5(0xFF, 0xAA, 0x01, 0x08, 0x00);
    system_delay_ms(3000);

    // 3. 保存设置
    imuSave();
}

// ★ Z 轴置零
void imuZeroZ(void)
{
    // 1. 解锁，延时 200ms
    imuUnlock();
    system_delay_ms(200);

    // 2. Z 轴置零，等待 3 秒：FF AA 01 04 00
    imuSend5(0xFF, 0xAA, 0x01, 0x04, 0x00);
    system_delay_ms(3000);

    // 3. 保存设置
    imuSave();
}
