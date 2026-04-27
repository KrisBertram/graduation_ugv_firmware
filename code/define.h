#ifndef CODE_DEFINE_H_
#define CODE_DEFINE_H_

#define MAX(a, b)           ((a) > (b) ? (a) : (b))
#define MIN(a, b)           ((a) < (b) ? (a) : (b))
#define CLIP(x, min, max)   ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

/* IMU */
#define HWT606_UART         (UART_3)            // HWT606 的串口中断号
#define HWT606_RX           (UART3_TX_P15_6)    // HWT606 RX 串口引脚连接到单片机此处，该引脚实际上是单片机 TX 引脚
#define HWT606_TX           (UART3_RX_P15_7)    // HWT606 TX 串口引脚连接到单片机此处，该引脚实际上是单片机 RX 引脚
#define HWT606_BAUD         (921600)            // HWT606 串口波特率，默认 921600 bps

/* 电机 */
#define PIT_MOTOR           (CCU61_CH0)         // 电机的周期中断定时器，与编码器共用定时器
#define MOTOR_CH1           (ATOM0_CH7_P02_7)   // 电机 PWM 通道 1
#define MOTOR_CH2           (ATOM0_CH6_P02_6)   // 电机 PWM 通道 2
#define MOTOR_FREQ          (17000)             // 电机频率

/* 舵机 */
#define PIT_STEER           (CCU61_CH1)         // 舵机的周期中断定时器
#define STEER_CH1           (ATOM1_CH1_P33_9)   // 舵机对应引脚
#define STEER_FREQ          (125)               // 舵机频率，注意范围 50-300，最佳 125Hz
#define SERVO_DUTY_R        (625)               // 舵机【脉宽范围】所允许的最右侧（占空比）
#define SERVO_DUTY_M        (1875)              // 舵机【脉宽范围】所允许的最中位（占空比）
#define SERVO_DUTY_L        (3125)              // 舵机【脉宽范围】所允许的最左侧（占空比）
#define STEER_DUTY_R        (1375)              // 舵机【活动范围】最右侧（占空比）
#define STEER_DUTY_M        (1887)              // 舵机【活动范围】最中位（占空比）
#define STEER_DUTY_L        (2400)              // 舵机【活动范围】最左侧（占空比）
#define STEER_DUTY_MIN      (STEER_DUTY_R)      // 舵机【活动范围】的数值下限
#define STEER_DUTY_MAX      (STEER_DUTY_L)      // 舵机【活动范围】的数值上限
// PWM 值 = (脉宽 / 周期) * PWM_DUTY_MAX；duty (0%-50%-100%) ~ 脉宽 (0.5-1.5-2.5) ~ PWM (625-1875-3125)
#define STEER_MS2PWM(ms)    ((uint32)(((float)(ms) / (1000f / (float)STEER_FREQ)) * (float)PWM_DUTY_MAX)) // 脉宽 ~ PWM
#define STEER_DUTY2PWM(x)   (STEER_MS2PWM(0.5f + ((float)(x) / 100.0f) * 2.0f)) // duty ~ PWM
#if (STEER_FREQ < 50 || STEER_FREQ > 300)
    #error "STEER_FREQ ERROE!"
#endif

/* 编码器 */
#define TIM_ENCODER         (TIM2_ENCODER)              // 带方向编码器对应使用的 GPT12 通用定时器中断
#define ENCODER_PULSE       (TIM2_ENCODER_CH1_P33_7)    // 编码器 PULSE 对应的引脚
#define ENCODER_DIR         (TIM2_ENCODER_CH2_P33_6)    // 编码器 DIR 对应的引脚

/* GPIO */
#define BUZZER              (P33_10)            // 主板上蜂鸣器对应引脚
#define LED1                (P20_6)             // 指示灯 1 对应引脚
#define LED2                (P20_7)             // 指示灯 2 对应引脚
#define LED3                (P20_8)             // 指示灯 3 对应引脚
#define LED4                (P20_9)             // 指示灯 4 对应引脚

/* 按键 */
/***********************************************************************************
 *  32.4    22.2    22.3    |   key01    key02    key03   |   待定     待定     待定
 *  33.13   22.1    21.2    |   key04    key05    key06   |   待定     待定     待定
 *  33.12   22.0    21.3    |   key07    key08    key09   |   待定     待定     待定
 *  33.11   23.1    21.4    |   key10    key11    key12   |   待定     待定     待定
 ***********************************************************************************/
#define KEY01               (P32_4)
#define KEY02               (P22_2)
#define KEY03               (P22_3)
#define KEY04               (P33_13)
#define KEY05               (P22_1)
#define KEY06               (P21_2)
#define KEY07               (P33_12)
#define KEY08               (P22_0)
#define KEY09               (P21_3)
#define KEY10               (P33_11)
#define KEY11               (P23_1)
#define KEY12               (P21_4)

/* DUMBORC X6F 遥控器 */
//  ------------------------------
//  |               1    * * *   |       ch1 vcc gnd
//  |     X6F       2    * * *   |       ch2 vcc gnd
//  |               3    * * *   |       ch3 vcc gnd
//  |               4    * * *   |       ch4 vcc gnd
//  |               5    * * *   |       ch5 vcc gnd
//  |               6    * * *   |       ch6 vcc gnd
//  ------------------------------
#define PIT_X6F             (CCU60_CH1)         // 遥控器的周期中断定时器
#define X6F_CH1             (P21_5)             // 方向舵通道
#define X6F_CH2             (P21_6)             // 油门通道
#define X6F_CH3             (P21_7)             // 指示按钮通道
#define X6F_CH4             (P20_0)             // 拨动挡位通道
#define X6F_CH5             (P20_2)             // 旋钮通道 1
#define X6F_CH6             (P20_3)             // 旋钮通道 2

/* WIFI 模块 */
#define ATK_MW8266D_UART    (UART_0)            // 定义 WiFi 使用的串口
#define ATK_MW8266D_TX      (UART0_RX_P14_1)    // 连接 WIFI 模块 TX
#define ATK_MW8266D_RX      (UART0_TX_P14_0)    // 连接 WIFI 模块 RX
#define ATK_MW8266D_BAUD    (115200)            // 模块工作波特率
#define ATK_MW8266D_RST_PIN (P33_5)             // 定义硬件复位引脚

#define WIFI_SSID       ("uavap_tsang")     // WiFi SSID
#define WIFI_PASSWORD   ("888888887")       // WiFi 密码
#define TCP_SERVER_IP   ("10.190.49.61")    // TCP 服务器 IP
#define TCP_SERVER_PORT ("5001")            // TCP 服务器端口

#endif /* CODE_DEFINE_H_ */
