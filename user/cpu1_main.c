#include "zf_common_headfile.h"
#include "define.h"
#include "motion.h"
#include "hwt606.h"
#include "mmath.h"
#include "wifi.h"
#include "gui.h"
#include "path_follow.h"
#pragma section all "cpu1_dsram"

void core1_main(void)
{
    disable_Watchdog();         // 关闭看门狗
    interrupt_global_enable(0); // 打开全局中断

    gpio_init(BUZZER, GPO, GPIO_LOW, GPO_PUSH_PULL); // 初始化蜂鸣器
    ledInit(); // 初始化指示灯输出，默认低电平，推挽输出模式
    x6fInit(); // 初始化遥控器，默认低电平，上拉输入

    cpu_wait_event_ready();     // 等待所有核心初始化完毕

    while (TRUE)
    {
        if (atk_mw8266d_ready()) gpio_set_level(LED1, 1);
        if (atk_mw8266d_info.isSerianet) gpio_set_level(LED2, 1);

        x6fCh3Handle();
        x6fCh4Handle();

        if (rc_enable_flag) // 如果遥控器使能
        {
            if (pathFollowerIsRunning()) pathFollowerStop(); // 遥控器接管最高优先级，自动控制必须退出
            motor_duty = (float)mapTri(x6f_channel[1].out, 125, 174, 200, -18, 0, 18);
            steer_duty = (float)mapTri(x6f_channel[0].out, 139, 150, 162, STEER_DUTY_R, STEER_DUTY_M, STEER_DUTY_L);
        }
    }
}

#pragma section all restore /* section all "cpu1_dsram" 与 section all restore 语句之间的全局变量都放在 CPU1 的 RAM 中 */
