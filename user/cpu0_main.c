#include "zf_common_headfile.h"
#include "wit_c_sdk.h"
#include "define.h"
#include "motion.h"
#include "hwt606.h"
#include "mmath.h"
#include "wifi.h"
#include "wifi_packet.h"
#include "ins.h"
#include "gui.h"
#include "path_follow.h"
#pragma section all "cpu0_dsram"

int core0_main(void)
{
    clock_init(); // 获取时钟频率
    debug_init(); // 初始化默认调试串口

    /* 舵机初始化 */
    pwm_init(STEER_CH1, STEER_FREQ, STEER_DUTY_M); // 初始化舵机 PWM 通道 1
    pit_ms_init(PIT_STEER, 2); // 初始化 CCU61_CH1 作为舵机的周期中断定时器

    /* 电机初始化 */
    pwm_init(MOTOR_CH1, MOTOR_FREQ, 0); // 初始化电机 PWM 通道 1，频率 17KHz，初始占空比 0%
    pwm_init(MOTOR_CH2, MOTOR_FREQ, 0); // 初始化电机 PWM 通道 2，频率 17KHz，初始占空比 0%
    pit_ms_init(PIT_MOTOR, 5); // 初始化 CC61_CH0 作为电机的周期中断定时器，与编码器共用定时器

    /* 编码器初始化 */
    encoder_dir_init(TIM_ENCODER, ENCODER_PULSE, ENCODER_DIR);

    /* IMU 初始化 */
    uart_init(HWT606_UART, HWT606_BAUD, HWT606_RX, HWT606_TX);
    uart_rx_interrupt(HWT606_UART, 0);
    WitInit(WIT_PROTOCOL_NORMAL, 0x50);             // 初始化 WIT 协议栈
    WitSerialWriteRegister(WitSerialWrite);         // 注册串口发送函数（发给 IMU）
    WitRegisterCallBack(WitRegUpdateHandler);       // 注册数据更新回调函数
    WitDelayMsRegister(WitDelayms);                 // 注册延时函数
    hwtInitialized = TRUE;
    uart_rx_interrupt(HWT606_UART, 1);

    /* WiFi 初始化 */
    zf_log(atk_mw8266d_init() == ATK_MW8266D_EOK, "wifi init failed");
    zf_log(atk_mw8266d_echo_config(0) == ATK_MW8266D_EOK, "wifi echo config failed");
    atk_mw8266d_info.isPowerOn = 1;

    /* 连接到 AP TCP 服务端 */
    if (atk_mw8266d_info.isPowerOn)
    {
        uint8 wifi_ret;

        if (!atk_mw8266d_info.isModeSet) // STA 模式，允许连接失败
        {
            wifi_ret = atk_mw8266d_set_mode(1); // 设置为 STA 模式
            if (wifi_ret == ATK_MW8266D_EOK) atk_mw8266d_info.isModeSet = 1;
            else atk_mw8266d_info.isModeSet = 0;
        }

        if (atk_mw8266d_info.isModeSet && !atk_mw8266d_info.isWiFiConnected)
            atk_mw8266d_sw_reset(); // 软件复位，重启生效

        if (!atk_mw8266d_info.isWiFiConnected) // 连接到 AP，初始化时允许连接失败
        {
            wifi_ret = atk_mw8266d_join_ap(WIFI_SSID, WIFI_PASSWORD); // 加入 AP
            if (wifi_ret == ATK_MW8266D_EOK) atk_mw8266d_info.isWiFiConnected = 1;
            else atk_mw8266d_info.isWiFiConnected = 0;
        }

        if (atk_mw8266d_info.isWiFiConnected && !atk_mw8266d_info.isTcpConnected)
            atk_mw8266d_mux_config(0); // 设置单连接模式

        if (!atk_mw8266d_info.isTcpConnected) // 连接到 TCP 服务端，初始化时允许连接失败
        {
            wifi_ret = atk_mw8266d_connect_tcp_server(TCP_SERVER_IP, TCP_SERVER_PORT); // 连接到 TCP 服务器
            if (wifi_ret == ATK_MW8266D_EOK) atk_mw8266d_info.isTcpConnected = 1;
            else atk_mw8266d_info.isTcpConnected = 0;
        }

        if (atk_mw8266d_ready()) // 如果成功连上 TCP 服务器，则允许进入透传
        {
            wifi_ret = atk_mw8266d_enter_unvarnished(); // 进入透传模式
            if (wifi_ret == ATK_MW8266D_EOK) atk_mw8266d_info.isSerianet = 1;
            else atk_mw8266d_info.isSerianet = 0;
        }

        if (atk_mw8266d_ready() && atk_mw8266d_info.isSerianet) buzzerBeep(BUZZER_FAST); // 指示 WiFi 成功连接
    }

    /* IMU 校准 */
    system_delay_ms(200);   // 避免开机抖动
    buzzerBeep(BUZZER_SS);  // 短鸣两声
    imuZeroXY();            // X、Y 轴角度置零
    imuZeroZ();             // Z 轴角度置零

    /* 航位解算状态初始化：开机车头方向作为全局 +X 方向 */
    carPoseInit(&carPose, 0.0f);

    cpu_wait_event_ready(); // 等待所有核心初始化完毕
    buzzerBeep(BUZZER_L);   // 长鸣指示初始化完成

    while (TRUE)
    {
        if (hwtUpdate)
        {
            uint8 update_flags = (uint8)hwtUpdate; // 记录本轮处理前的 IMU 更新标志，避免清掉处理期间新来的标志

            imu_dat.ax = sReg[AX] / 32768.0f * 16.0f;
            imu_dat.ay = sReg[AY] / 32768.0f * 16.0f;
            imu_dat.az = sReg[AZ] / 32768.0f * 16.0f;

            imu_dat.gx = sReg[GX] / 32768.0f * 2000.0f;
            imu_dat.gy = sReg[GY] / 32768.0f * 2000.0f;
            imu_dat.gz = sReg[GZ] / 32768.0f * 2000.0f;

            imu_dat.Q0 = sReg[q0] / 32768.0f;
            imu_dat.Q1 = sReg[q1] / 32768.0f;
            imu_dat.Q2 = sReg[q2] / 32768.0f;
            imu_dat.Q3 = sReg[q3] / 32768.0f;

            imu_dat.roll = sReg[Roll] / 32768.0f * 180.0f;
            imu_dat.pitch = sReg[Pitch] / 32768.0f * 180.0f;
            imu_dat.yaw = sReg[Yaw] / 32768.0f * 180.0f;

            uint16 yymmReg = (uint16)sReg[YYMM];
            imu_dat.yy = yymmReg & 0x00FF;          // 低 8 位: 年份
            imu_dat.mm = (yymmReg >> 8) & 0x00FF;   // 高 8 位: 月份

            uint16 ddhhReg = (uint16)sReg[DDHH];
            imu_dat.dd = ddhhReg & 0x00FF;          // 低 8 位: 日期
            imu_dat.hh = (ddhhReg >> 8) & 0x00FF;   // 高 8 位: 小时

            uint16 mmssReg = (uint16)sReg[MMSS];
            imu_dat.mn = mmssReg & 0x00FF;          // 低 8 位: 分钟
            imu_dat.ss = (mmssReg >> 8) & 0x00FF;   // 高 8 位: 秒
            imu_dat.ms = (uint16)sReg[MS];          // 毫秒
            imu_dat.timestamp = imuGetTimeStamp(&imu_dat); // 时间戳

            if (BIT_IS_TRUE(update_flags, UPDATE_TIME)) // 仅在 IMU 时间戳更新后推进一次航位解算
            {
                float dt = 0.0f;
                if (imu_dat.timestampLast < 1e-4f)
                {
                    dt = 0.0f; // 第一帧，无 dt
                }
                else
                {
                    dt = imu_dat.timestamp - imu_dat.timestampLast;
                    if (dt < 0.0f) // 考虑跨小时、跨午夜
                    {
                        dt += 24.0f * 3600.0f;
                    }
                }
                imu_dat.timestampLast = imu_dat.timestamp;

                /* 编码器速度 + 舵机角 + 陀螺 Z 轴 + AHRS 航向角互补融合，更新小车全局位姿 */
                carPoseUpdate(&carPose, encoder_v, steer_duty, imu_dat.gz, imu_dat.yaw, dt);

                /* 自动轨迹跟踪只在主循环里更新控制量，不在中断里做复杂浮点控制 */
                pathFollowerUpdate(&carPose, dt, rc_enable_flag);
            }

            hwtUpdate = (uint8)(hwtUpdate & (uint8)(~update_flags));
        }

        if (wifi_tx_timer >= 5) // 每 10ms 发送一次
        {
            wifi_tx_timer = 0;

            /* 如果要改变上行数据包格式，先进入 wifi_packet.h 修改数据包结构体 SendPacket_t 的格式 */
            /* 然后分别修改下面的数据包内容 send_packet 和字段描述表内容 send_desc */
            SendPacket_t send_packet = {
                .speed = carPose.speed,
                .distance = carPose.distance,
                .yaw = carPose.yaw_deg,
                .pitch = imu_dat.pitch,
                .roll = imu_dat.roll,
                .pos_x = carPose.x,
                .pos_y = carPose.y,
                .pos_z = 0,
                .action = 1
            };

            const FieldDesc_t send_desc[] = {
                { &send_packet.speed,    TYPE_FLOAT32 },
                { &send_packet.distance, TYPE_FLOAT32 },
                { &send_packet.yaw,      TYPE_FLOAT32 },
                { &send_packet.pitch,    TYPE_FLOAT32 },
                { &send_packet.roll,     TYPE_FLOAT32 },
                { &send_packet.pos_x,    TYPE_FLOAT32 },
                { &send_packet.pos_y,    TYPE_FLOAT32 },
                { &send_packet.pos_z,    TYPE_FLOAT32 },
                { &send_packet.action,   TYPE_UINT16  }
            };

            serial_datapacket_send(0x01, send_desc, ARRAY_SIZE(send_desc));
        }
    }
}

#pragma section all restore /* section all "cpu0_dsram" 与 section all restore 语句之间的全局变量都放在 CPU0 的 RAM 中 */
