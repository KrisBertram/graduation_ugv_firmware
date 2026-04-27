#ifndef CODE_GUI_H_
#define CODE_GUI_H_
#include "zf_driver_gpio.h"

#define kb_beep(PIN) gpio_set_level(BUZZER_PIN, 1); system_delay_ms(300); gpio_set_level(BUZZER_PIN, 0)

typedef enum {
    KEY_NONE,
    KEY_SHORT,
    KEY_LONG
} KeyEvent_t;

typedef enum {
    BUZZER_S,   // 短鸣
    BUZZER_L,   // 长鸣
    BUZZER_SS,  // 双短鸣
    BUZZER_SSS, // 三短鸣
    BUZZER_SSL, // 短短长
    BUZZER_LSS, // 长短短
    BUZZER_FAST,// 快速短鸣
} BuzzerMode_t;

typedef struct {
    gpio_pin_enum pin;
    uint8 state;
    uint8 state_last;
    uint8 counter;          // 消抖计数
    uint16 long_counter;    // 长按计数
    uint16 repeat_counter;  // 连发计数
} Key_t;

typedef struct {
    int16 pin;          // 引脚
    int16 count;        // 高电平计数变量
    int16 out;          // 高电平计数输出值
} X6F_Channel_t;

extern volatile uint8 key_scan_flag;
extern uint8 rc_enable_flag, rc_init_flag;
extern Key_t key[12];
extern X6F_Channel_t x6f_channel[6];

void ledInit(void);
void keyInit(void);
void x6fInit(void);

KeyEvent_t keyScan(Key_t* key);
void x6fScan(void);
void x6fCh3Handle(void);
void x6fCh4Handle(void);
void buzzerBeep(BuzzerMode_t mode);

#endif /* CODE_GUI_H_ */
