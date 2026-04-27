#include "zf_common_headfile.h"
#include "define.h"
#include "motion.h"
#include "hwt606.h"
#include "mmath.h"
#include "wifi.h"
#include "gui.h"
#pragma section all "cpu1_dsram"

void core1_main(void)
{
    disable_Watchdog();         // 关闭看门狗
    interrupt_global_enable(0); // 打开全局中断

    gpio_init(BUZZER, GPO, GPIO_LOW, GPO_PUSH_PULL); // 初始化蜂鸣器
    ledInit(); // 初始化指示灯输出，默认低电平，推挽输出模式
    keyInit(); // 初始化主板按键输入，默认高电平，上拉输入
    x6fInit(); // 初始化遥控器，默认低电平，上拉输入

    cpu_wait_event_ready();     // 等待所有核心初始化完毕

    while (TRUE)
    {
        if (key_scan_flag >= 2) // 保证按键轮询 4 毫秒触发一次
        {
            key_scan_flag = 0;

            KeyEvent_t kev3 = keyScan(&key[2]);
            KeyEvent_t kev6 = keyScan(&key[5]);
            KeyEvent_t kev9 = keyScan(&key[8]);
            KeyEvent_t kev12 = keyScan(&key[11]);

            if(kev3 == KEY_SHORT) // WiFi 请求连接
            {
                buzzerBeep(BUZZER_S);
                static uint8 wifi_ret;

                if (!atk_mw8266d_info.isModeSet) // STA 模式，允许连接失败
                {
                    wifi_ret = atk_mw8266d_set_mode(1);
                    if (wifi_ret == ATK_MW8266D_EOK) atk_mw8266d_info.isModeSet = 1;
                    else atk_mw8266d_info.isModeSet = 0;
                }

                if (atk_mw8266d_info.isModeSet && !atk_mw8266d_info.isWiFiConnected)
                    atk_mw8266d_sw_reset(); // 软件复位，重启生效

                if (!atk_mw8266d_info.isWiFiConnected) // 连接到 AP，初始化时允许连接失败
                {
                    wifi_ret = atk_mw8266d_join_ap(WIFI_SSID, WIFI_PASSWORD);
                    if (wifi_ret == ATK_MW8266D_EOK) atk_mw8266d_info.isWiFiConnected = 1;
                    else atk_mw8266d_info.isWiFiConnected = 0;
                }

                if (atk_mw8266d_info.isWiFiConnected && !atk_mw8266d_info.isTcpConnected)
                    atk_mw8266d_mux_config(0); // 设置单连接模式

                if (!atk_mw8266d_info.isTcpConnected) // 连接到 TCP 服务端，初始化时允许连接失败
                {
                    wifi_ret = atk_mw8266d_connect_tcp_server(TCP_SERVER_IP, TCP_SERVER_PORT);
                    if (wifi_ret == ATK_MW8266D_EOK) atk_mw8266d_info.isTcpConnected = 1;
                    else atk_mw8266d_info.isTcpConnected = 0;
                }

                if (atk_mw8266d_ready()) // 如果成功连上 TCP 服务器，则允许进入透传
                {
                    wifi_ret = atk_mw8266d_enter_unvarnished();
                    if (wifi_ret == ATK_MW8266D_EOK) atk_mw8266d_info.isSerianet = 1;
                    else atk_mw8266d_info.isSerianet = 0;
                }

                if (atk_mw8266d_ready() && atk_mw8266d_info.isSerianet) buzzerBeep(BUZZER_FAST); // 指示 WiFi 成功连接
            }

            if(kev6 == KEY_SHORT)
            {
                steer_duty += 25;
            }
            else if(kev6 == KEY_LONG)
            {
                steer_duty += 100;
            }

            if(kev9 == KEY_SHORT)
            {
                steer_duty -= 25;
            }
            else if(kev9 == KEY_LONG)
            {
                steer_duty -= 100;
            }

            if (kev12 == KEY_SHORT)
            {
                system_delay_ms(1000);  // 避免按键抖动
                buzzerBeep(BUZZER_SS);  // 短鸣两声
                imuCalibrateAcc();      // 校准加速度计
                imuZeroXY();            // X、Y 轴角度置零
                imuZeroZ();             // Z 轴角度置零
                buzzerBeep(BUZZER_L);   // 指示初始化完成
            }
        }

        if (atk_mw8266d_ready()) gpio_set_level(LED1, 1);
        if (atk_mw8266d_info.isSerianet) gpio_set_level(LED2, 1);

        x6fCh3Handle();
        x6fCh4Handle();

        if (rc_enable_flag) // 如果遥控器使能
        {
            motor_duty = (float)mapTri(x6f_channel[1].out, 125, 174, 200, -18, 0, 18);
            steer_duty = (float)mapTri(x6f_channel[0].out, 139, 150, 162, STEER_DUTY_R, STEER_DUTY_M, STEER_DUTY_L);
        }
    }
}

#pragma section all restore /* section all "cpu1_dsram" 与 section all restore 语句之间的全局变量都放在 CPU1 的 RAM 中 */
