#include "zf_common_headfile.h"
#include "define.h"
#include "gui.h"

#define KEY_DEBOUNCE    4       // 按键轮询消抖次数
#define KEY_LONG_CNT    150     // 按键轮询长按计数，轮询周期 4ms -> 0.6s
#define KEY_REPEAT_CNT  10      // 长按连发周期，轮询周期 4ms -> 每 40ms 连发一次

volatile uint8 key_scan_flag = 0;
uint8 rc_init_flag = FALSE;     // 遥控器是否完成初始化
uint8 rc_enable_flag = FALSE;   // 是否使能遥控器操作，通过遥控器 CH3 控制，按下灯亮 = 使能

Key_t key[12] = {
    { KEY01, 1, 1, 0, 0, 0 },
    { KEY02, 1, 1, 0, 0, 0 },
    { KEY03, 1, 1, 0, 0, 0 },
    { KEY04, 1, 1, 0, 0, 0 },
    { KEY05, 1, 1, 0, 0, 0 },
    { KEY06, 1, 1, 0, 0, 0 },
    { KEY07, 1, 1, 0, 0, 0 },
    { KEY08, 1, 1, 0, 0, 0 },
    { KEY09, 1, 1, 0, 0, 0 },
    { KEY10, 1, 1, 0, 0, 0 },
    { KEY11, 1, 1, 0, 0, 0 },
    { KEY12, 1, 1, 0, 0, 0 }
};

X6F_Channel_t x6f_channel[6] = {
    { X6F_CH1, 0, 0 },
    { X6F_CH2, 0, 0 },
    { X6F_CH3, 0, 0 },
    { X6F_CH4, 0, 0 },
    { X6F_CH5, 0, 0 },
    { X6F_CH6, 0, 0 }
};

// 初始化 LED 指示灯
void ledInit(void)
{
    gpio_init(LED1, GPO, GPIO_LOW, GPO_PUSH_PULL); // 初始化 LED1 输出，默认低电平，推挽输出模式
    gpio_init(LED2, GPO, GPIO_LOW, GPO_PUSH_PULL); // 初始化 LED2 输出，默认低电平，推挽输出模式
    gpio_init(LED3, GPO, GPIO_LOW, GPO_PUSH_PULL); // 初始化 LED3 输出，默认低电平，推挽输出模式
    gpio_init(LED4, GPO, GPIO_LOW, GPO_PUSH_PULL); // 初始化 LED4 输出，默认低电平，推挽输出模式
}

// 初始化主板按键输入
void keyInit(void)
{
    gpio_init(KEY01, GPI, GPIO_HIGH, GPI_PULL_UP); // 初始化 KEY1 输入，默认高电平，上拉输入
    gpio_init(KEY02, GPI, GPIO_HIGH, GPI_PULL_UP); // 初始化 KEY2 输入，默认高电平，上拉输入
    gpio_init(KEY03, GPI, GPIO_HIGH, GPI_PULL_UP); // 初始化 KEY3 输入，默认高电平，上拉输入
    gpio_init(KEY04, GPI, GPIO_HIGH, GPI_PULL_UP); // 初始化 KEY4 输入，默认高电平，上拉输入
    gpio_init(KEY05, GPI, GPIO_HIGH, GPI_PULL_UP); // 初始化 KEY5 输入，默认高电平，上拉输入
    gpio_init(KEY06, GPI, GPIO_HIGH, GPI_PULL_UP); // 初始化 KEY6 输入，默认高电平，上拉输入
    gpio_init(KEY07, GPI, GPIO_HIGH, GPI_PULL_UP); // 初始化 KEY7 输入，默认高电平，上拉输入
    gpio_init(KEY08, GPI, GPIO_HIGH, GPI_PULL_UP); // 初始化 KEY8 输入，默认高电平，上拉输入
    gpio_init(KEY09, GPI, GPIO_HIGH, GPI_PULL_UP); // 初始化 KEY9 输入，默认高电平，上拉输入
    gpio_init(KEY10, GPI, GPIO_HIGH, GPI_PULL_UP); // 初始化 KEY10输入，默认高电平，上拉输入
    gpio_init(KEY11, GPI, GPIO_HIGH, GPI_PULL_UP); // 初始化 KEY11输入，默认高电平，上拉输入
    gpio_init(KEY12, GPI, GPIO_HIGH, GPI_PULL_UP); // 初始化 KEY12输入，默认高电平，上拉输入
}

// 按键轮询
KeyEvent_t keyScan(Key_t* key)
{
    KeyEvent_t event = KEY_NONE;
    uint8 current_state = gpio_get_level(key->pin);

    /* 消抖处理 */
    if(current_state != key->state_last)
    {
        key->counter = 0; // 状态变化，重置计数
    }
    else
    {
        if(key->counter < KEY_DEBOUNCE)
        {
            key->counter++;
        }
        else
        {
            if(current_state != key->state)
            {
                key->state = current_state;
                if(current_state == 0) // 按键按下
                {
                    key->long_counter = 0; // 按下开始计长按
                }
                else
                {
                    if(key->long_counter < KEY_LONG_CNT)
                    {
                        event = KEY_SHORT; // 松开且未长按
                    }
                    key->long_counter = 0;
                }
            }
        }
    }

    key->state_last = current_state;

    /* 长按判断 */
    if(key->state == 0)
    {
        ++key->long_counter;

        // 长按第一次触发
        if(key->long_counter == KEY_LONG_CNT)
        {
            event = KEY_LONG;
        }

        // 长按连发
        if(key->long_counter > KEY_LONG_CNT)
        {
            ++key->repeat_counter;
            if(key->repeat_counter >= KEY_REPEAT_CNT)
            {
                key->repeat_counter = 0;
                event = KEY_LONG; // 再次触发长按事件，实现连发
            }
        }
    }

    return event;
}

// 初始化遥控器
void x6fInit(void)
{
    // 初始化接收机引脚，默认低电平，上拉输入
    gpio_init(X6F_CH1, GPI, GPIO_LOW, GPI_PULL_UP);
    gpio_init(X6F_CH2, GPI, GPIO_LOW, GPI_PULL_UP);
    gpio_init(X6F_CH3, GPI, GPIO_LOW, GPI_PULL_UP);
    gpio_init(X6F_CH4, GPI, GPIO_LOW, GPI_PULL_UP);
    gpio_init(X6F_CH5, GPI, GPIO_LOW, GPI_PULL_UP);
    gpio_init(X6F_CH6, GPI, GPIO_LOW, GPI_PULL_UP);
    pit_us_init(PIT_X6F, 10); // 初始化周期中断定时器，用于读取接收机各通道引脚高电平时间
    rc_init_flag = TRUE;
}

// 遥控器通道扫描
void x6fScan(void)
{
    for (uint8 i = 0, ie = 6; i < ie; ++i)
    {
        if(gpio_get_level(x6f_channel[i].pin))
        {
            ++x6f_channel[i].count;
        }
        else if(x6f_channel[i].count > 0)
        {
            x6f_channel[i].out = x6f_channel[i].count;
            x6f_channel[i].count = 0;
        }
    }
}

// 使用遥控器 Channel 3 切换两个档位（按下灯亮、未按下灯灭）
void x6fCh3Handle(void)
{
    if (!rc_init_flag) return;

    // 如果 CH3 处于未按下状态（灯灭）
    if (x6f_channel[2].out < 140)
    {
        rc_enable_flag = FALSE;
    }
    // 如果 CH3 处于按下状态（灯亮）
    else
    {
        rc_enable_flag = TRUE;
    }
}

// 使用遥控器 Channel 4 切换三个档位（左位、中位、右位）
void x6fCh4Handle(void)
{
    if (!rc_init_flag || !rc_enable_flag) return;

    // 如果 CH4 打到最左边
    if(x6f_channel[3].out < 140)
    {

    }
    // 如果 CH4 打到最右边
    else if(x6f_channel[3].out > 160)
    {

    }
    // 如果 CH4 打到在中间
    else
    {

    }
}

static inline void buzzerOn(uint32 duration_ms)
{
    gpio_set_level(BUZZER, GPIO_HIGH);
    system_delay_ms(duration_ms);
    gpio_set_level(BUZZER, GPIO_LOW);
}

void buzzerBeep(BuzzerMode_t mode)
{
    switch(mode)
    {
        case BUZZER_S:      // 短
            buzzerOn(100);
            break;

        case BUZZER_L:      // 长
            buzzerOn(300);
            break;

        case BUZZER_SS:     // 短短
            buzzerOn(100);
            system_delay_ms(100);
            buzzerOn(100);
            break;

        case BUZZER_SSS:    // 短短短
            for(int i = 0; i < 3; ++i)
            {
                buzzerOn(100);
                if(i < 2) system_delay_ms(100);
            }
            break;

        case BUZZER_SSL:    // 短短长
            buzzerOn(100);
            system_delay_ms(100);
            buzzerOn(100);
            system_delay_ms(100);
            buzzerOn(300);
            break;

        case BUZZER_LSS:    // 长短短
            buzzerOn(300);
            system_delay_ms(100);
            buzzerOn(100);
            system_delay_ms(100);
            buzzerOn(100);
            break;

        case BUZZER_FAST:   // 快闪 5 次
            for(int i = 0; i < 5; i++)
            {
                buzzerOn(100);
                system_delay_ms(100);
            }
            break;

        default:
            break;
    }
}
