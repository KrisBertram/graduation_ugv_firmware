#include "zf_common_headfile.h"
#include "define.h"
#include "menu_gui.h"
#pragma section all "cpu2_dsram"

void core2_main(void)
{
    disable_Watchdog();         // 关闭看门狗
    interrupt_global_enable(0); // 打开全局中断

    /* 屏幕与菜单初始化。按键扫描也由 CPU2 统一持有，避免跨核抢事件。 */
    ips200_set_dir(IPS200_PORTAIT);
    ips200_set_color(RGB565_CYAN, RGB565_BLACK);
    ips200_init(IPS200_TYPE_PARALLEL8);
    menuGuiInit();

    cpu_wait_event_ready();     // 等待所有核心初始化完毕

    while (TRUE)
    {
        menuGuiTask();
    }
}

#pragma section all restore /* section all "cpu2_dsram" 与 section all restore 语句之间的全局变量都放在 CPU2 的 RAM 中 */
