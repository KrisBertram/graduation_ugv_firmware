#ifndef CODE_WIFI_H_
#define CODE_WIFI_H_

#define ATK_MW8266D_UART_RX_BUF_SIZE (256) // 定义接收缓存区大小
#define ATK_MW8266D_UART_TX_BUF_SIZE (128) // 定义发送缓存区大小

/* 错误代码 */
#define ATK_MW8266D_EOK         (0)         /* 没有错误 */
#define ATK_MW8266D_ERROR       (1)         /* 通用错误 */
#define ATK_MW8266D_ETIMEOUT    (2)         /* 超时错误 */
#define ATK_MW8266D_EINVAL      (3)         /* 参数错误 */

/* 硬件复位 */
#define ATK_MW8266D_RST(x) gpio_set_level(ATK_MW8266D_RST_PIN, x)

typedef struct {
    uint8_t buf[ATK_MW8266D_UART_RX_BUF_SIZE];  /* 帧接收缓冲 */
    struct
    {
        uint16_t len    : 15;                   /* 帧接收长度，sta[14:0] */
        uint16_t finsh  : 1;                    /* 帧接收完成标志，sta[15] */
    } sta;                                      /* 帧状态信息 */
} atk_mw8266d_rx_buffer_t;                      /* ATK-MW8266D UART 接收帧缓冲信息结构体 */

typedef struct {
    uint8 isPowerOn;        // 模块是否已上电初始化完成
    uint8 isModeSet;        // 是否已设置 WiFi 模式
    uint8 isWiFiConnected;  // WiFi 是否连接
    uint8 isTcpConnected;   // 是否已建立 TCP 连接
    uint8 isSerianet;       // 处于透传模式
} atk_mw8266d_info_t;

extern atk_mw8266d_rx_buffer_t g_uart_rx_frame;
extern atk_mw8266d_info_t atk_mw8266d_info;
extern volatile uint8 wifi_tx_timer;

extern void atk_mw8266d_hw_reset(void);
extern void atk_mw8266d_uart_printf(char *fmt, ...);
extern void atk_mw8266d_uart_rx_restart(void);
extern uint8_t *atk_mw8266d_uart_rx_get_frame(void);
extern uint16_t atk_mw8266d_uart_rx_get_frame_len(void);
extern uint8_t atk_mw8266d_send_at_cmd(char *cmd, char *ack, uint32_t timeout);
extern uint8_t atk_mw8266d_at_test(void);
extern uint8_t atk_mw8266d_init(void);
extern uint8_t atk_mw8266d_restore(void);
extern uint8_t atk_mw8266d_set_mode(uint8_t mode);
extern uint8_t atk_mw8266d_sw_reset(void);
extern uint8_t atk_mw8266d_echo_config(uint8_t cfg);
extern uint8_t atk_mw8266d_join_ap(char *ssid, char *pwd);
extern uint8_t atk_mw8266d_mux_config(uint8_t cfg);
extern uint8_t atk_mw8266d_get_ip(char *buf);
extern uint8_t atk_mw8266d_connect_tcp_server(char *server_ip, char *server_port);
extern uint8_t atk_mw8266d_enter_unvarnished(void);
extern void atk_mw8266d_exit_unvarnished(void);
extern uint8_t atk_mw8266d_connect_atkcld(char *id, char *pwd);
extern uint8_t atk_mw8266d_disconnect_atkcld(void);

inline uint8 atk_mw8266d_ready(void)
{
    return atk_mw8266d_info.isPowerOn &&
           atk_mw8266d_info.isModeSet &&
           atk_mw8266d_info.isWiFiConnected &&
           atk_mw8266d_info.isTcpConnected;
}

#endif /* CODE_WIFI_H_ */
