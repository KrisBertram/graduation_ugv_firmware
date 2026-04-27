#include "zf_common_headfile.h"
#pragma section all "cpu3_dsram"

void core3_main(void)
{
    disable_Watchdog();                     // 关闭看门狗
    interrupt_global_enable(0);             // 打开全局中断

    cpu_wait_event_ready();                 // 等待所有核心初始化完毕
    while (TRUE)
    {

    }
}

#pragma section all restore
