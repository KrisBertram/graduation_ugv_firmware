#include "zf_common_headfile.h"
#include "define.h"
#include "wifi.h"

atk_mw8266d_rx_buffer_t g_uart_rx_frame = { { 0 }, { 0, 0 } };  /* ATK-MW8266D UART 接收帧缓冲信息结构体 */
static uint8_t g_uart_tx_buf[ATK_MW8266D_UART_TX_BUF_SIZE];     /* ATK-MW8266D UART 发送缓冲 */
atk_mw8266d_info_t atk_mw8266d_info = { 0 };

volatile uint8 wifi_tx_timer = 0; // 数据包发送计时器

// ATK-MW8266D 硬件复位
void atk_mw8266d_hw_reset(void)
{
    ATK_MW8266D_RST(0); system_delay_ms(100);
    ATK_MW8266D_RST(1); system_delay_ms(500);
}

/**
 * @brief   ATK-MW8266D 串口打印输出数据
 * @param   fmt: 待打印的数据
 */
void atk_mw8266d_uart_printf(char *fmt, ...)
{
    va_list ap;
    uint32 len;

    va_start(ap, fmt);
    vsprintf((char *)g_uart_tx_buf, fmt, ap);
    va_end(ap);

    len = strlen((const char *)g_uart_tx_buf);
    uart_write_buffer(ATK_MW8266D_UART, g_uart_tx_buf, len);
}

// ATK-MW8266D UART 重新开始接收数据
void atk_mw8266d_uart_rx_restart(void)
{
    memset(g_uart_rx_frame.buf, 0, g_uart_rx_frame.sta.len);
    g_uart_rx_frame.sta.len     = 0;
    g_uart_rx_frame.sta.finsh   = 0;
}

/**
 * @brief   获取 ATK-MW8266D UART 接收到的一帧数据
 * @param   无
 * @retval  NULL: 未接收到一帧数据
 *          其他: 接收到的一帧数据
 */
uint8_t *atk_mw8266d_uart_rx_get_frame(void)
{
    if (g_uart_rx_frame.sta.finsh == 1)
    {
        g_uart_rx_frame.buf[g_uart_rx_frame.sta.len] = '\0';
        return g_uart_rx_frame.buf;
    }

    return NULL;
}

/**
 * @brief   获取 ATK-MW8266D UART 接收到的一帧数据的长度
 * @param   无
 * @retval  0: 未接收到一帧数据
 *          其他: 接收到的一帧数据的长度
 */
uint16_t atk_mw8266d_uart_rx_get_frame_len(void)
{
    if (g_uart_rx_frame.sta.finsh == 1)
    {
        return g_uart_rx_frame.sta.len;
    }

    return 0;
}

/**
 * @brief   ATK-MW8266D 发送 AT 指令
 * @param   cmd    : 待发送的 AT 指令
 *          ack    : 等待的响应
 *          timeout: 等待超时时间
 * @retval  ATK_MW8266D_EOK     : 函数执行成功
 *          ATK_MW8266D_ETIMEOUT: 等待期望应答超时，函数执行失败
 */
uint8_t atk_mw8266d_send_at_cmd(char *cmd, char *ack, uint32_t timeout)
{
    uint8_t *ret = NULL;

    atk_mw8266d_uart_rx_restart();
    atk_mw8266d_uart_printf("%s\r\n", cmd);

    if ((ack == NULL) || (timeout == 0))
    {
        return ATK_MW8266D_EOK;
    }
    else
    {
        while (timeout > 0)
        {
            ret = atk_mw8266d_uart_rx_get_frame();
            if (ret != NULL)
            {
                if (strstr((const char *)ret, ack) != NULL)
                {
                    return ATK_MW8266D_EOK;
                }
                else
                {
                    atk_mw8266d_uart_rx_restart();
                }
            }
            timeout--;
            system_delay_ms(1);
        }

        return ATK_MW8266D_ETIMEOUT;
    }
}

/**
 * @brief   ATK-MW8266D AT 指令测试
 * @param   无
 * @retval  ATK_MW8266D_EOK  : AT 指令测试成功
 *          ATK_MW8266D_ERROR: AT 指令测试失败
 */
uint8_t atk_mw8266d_at_test(void)
{
    uint8_t ret;
    uint8_t i;

    for (i = 0; i < 10; ++i)
    {
        ret = atk_mw8266d_send_at_cmd("AT", "OK", 500);
        if (ret == ATK_MW8266D_EOK)
        {
            return ATK_MW8266D_EOK;
        }
    }

    return ATK_MW8266D_ERROR;
}

/**
 * @brief   ATK-MW8266D 初始化
 * @param   void
 * @retval  ATK_MW8266D_EOK  : ATK-MW8266D 初始化成功，函数执行成功
 *          ATK_MW8266D_ERROR: ATK-MW8266D 初始化失败，函数执行失败
 */
uint8_t atk_mw8266d_init(void)
{
    /* ATK-MW8266D 复位引脚初始化 */
    gpio_init(ATK_MW8266D_RST_PIN, GPO, 1, GPO_PUSH_PULL);

    /* ATK-MW8266D 硬件复位 */
    atk_mw8266d_hw_reset();

    /* ATK-MW8266D 串口初始化 */
    uart_init(ATK_MW8266D_UART, ATK_MW8266D_BAUD, ATK_MW8266D_RX, ATK_MW8266D_TX);
    uart_rx_interrupt(ATK_MW8266D_UART, 1);

    if (atk_mw8266d_at_test() != ATK_MW8266D_EOK)   /* ATK-MW8266D AT指令测试 */
    {
        return ATK_MW8266D_ERROR;
    }

    return ATK_MW8266D_EOK;
}

/**
 * @brief   ATK-MW8266D 恢复出厂设置
 * @param   无
 * @retval  ATK_MW8266D_EOK  : 恢复出场设置成功
 *          ATK_MW8266D_ERROR: 恢复出场设置失败
 */
uint8_t atk_mw8266d_restore(void)
{
    uint8_t ret;

    ret = atk_mw8266d_send_at_cmd("AT+RESTORE", "ready", 3000);
    if (ret == ATK_MW8266D_EOK)
    {
        return ATK_MW8266D_EOK;
    }

    return ATK_MW8266D_ERROR;
}

/**
 * @brief   设置 ATK-MW8266D 工作模式
 * @param   mode: 1，Station 模式
 *                2，AP 模式
 *                3，AP + Station 模式
 * @retval  ATK_MW8266D_EOK   : 工作模式设置成功
 *          ATK_MW8266D_ERROR : 工作模式设置失败
 *          ATK_MW8266D_EINVAL: mode 参数错误，工作模式设置失败
 */
uint8_t atk_mw8266d_set_mode(uint8_t mode)
{
    uint8_t ret;

    switch (mode)
    {
        case 1:
        {
            ret = atk_mw8266d_send_at_cmd("AT+CWMODE=1", "OK", 500);    /* Station 模式 */
            break;
        }
        case 2:
        {
            ret = atk_mw8266d_send_at_cmd("AT+CWMODE=2", "OK", 500);    /* AP 模式 */
            break;
        }
        case 3:
        {
            ret = atk_mw8266d_send_at_cmd("AT+CWMODE=3", "OK", 500);    /* AP + Station 模式 */
            break;
        }
        default:
        {
            return ATK_MW8266D_EINVAL;
        }
    }

    if (ret == ATK_MW8266D_EOK)
    {
        return ATK_MW8266D_EOK;
    }

    return ATK_MW8266D_ERROR;
}

/**
 * @brief   ATK-MW8266D 软件复位
 * @param   无
 * @retval  ATK_MW8266D_EOK  : 软件复位成功
 *          ATK_MW8266D_ERROR: 软件复位失败
 */
uint8_t atk_mw8266d_sw_reset(void)
{
    uint8_t ret;

    ret = atk_mw8266d_send_at_cmd("AT+RST", "OK", 500);
    if (ret == ATK_MW8266D_EOK)
    {
        system_delay_ms(1000);
        return ATK_MW8266D_EOK;
    }

    return ATK_MW8266D_ERROR;
}

/**
 * @brief   ATK-MW8266D 设置回显模式
 * @param   cfg: 0，关闭回显
 *               1，打开回显
 * @retval  ATK_MW8266D_EOK  : 设置回显模式成功
 *          ATK_MW8266D_ERROR: 设置回显模式失败
 */
uint8_t atk_mw8266d_echo_config(uint8_t cfg)
{
    uint8_t ret;

    switch (cfg)
    {
        case 0:
        {
            ret = atk_mw8266d_send_at_cmd("ATE0", "OK", 500);   /* 关闭回显 */
            break;
        }
        case 1:
        {
            ret = atk_mw8266d_send_at_cmd("ATE1", "OK", 500);   /* 打开回显 */
            break;
        }
        default:
        {
            return ATK_MW8266D_EINVAL;
        }
    }

    if (ret == ATK_MW8266D_EOK)
    {
        return ATK_MW8266D_EOK;
    }

    return ATK_MW8266D_ERROR;
}

/**
 * @brief   ATK-MW8266D 连接 WIFI
 * @param   ssid: WIFI 名称
 *          pwd : WIFI 密码
 * @retval  ATK_MW8266D_EOK  : WIFI 连接成功
 *          ATK_MW8266D_ERROR: WIFI 连接失败
 */
uint8_t atk_mw8266d_join_ap(char *ssid, char *pwd)
{
    uint8_t ret;
    char cmd[64];

    sprintf(cmd, "AT+CWJAP=\"%s\",\"%s\"", ssid, pwd);
    ret = atk_mw8266d_send_at_cmd(cmd, "WIFI GOT IP", 10000);
    if (ret == ATK_MW8266D_EOK)
    {
        return ATK_MW8266D_EOK;
    }

    return ATK_MW8266D_ERROR;
}

/**
 * @brief   ATK-MW8266D 开启单连接模式
 * @param   cfg: 0，单连接模式
 *               1，多连接模式
 * @retval  ATK_MW8266D_EOK  : 设置单连接模式成功
 *          ATK_MW8266D_ERROR: 设置单连接模式失败
 */
uint8_t atk_mw8266d_mux_config(uint8_t cfg)
{
    uint8_t ret;

    switch (cfg)
    {
        case 0:
        {
            ret = atk_mw8266d_send_at_cmd("AT+CIPMUX=0", "OK", 500);   /* 单连接模式 */
            break;
        }
        case 1:
        {
            ret = atk_mw8266d_send_at_cmd("AT+CIPMUX=1", "OK", 500);   /* 多连接模式 */
            break;
        }
        default:
        {
            return ATK_MW8266D_EINVAL;
        }
    }

    if (ret == ATK_MW8266D_EOK)
    {
        return ATK_MW8266D_EOK;
    }

    return ATK_MW8266D_ERROR;
}

/**
 * @brief   ATK-MW8266D 获取 IP 地址
 * @param   buf: IP 地址，需要 16 字节内存空间
 * @retval  ATK_MW8266D_EOK  : 获取 IP 地址成功
 *          ATK_MW8266D_ERROR: 获取 IP 地址失败
 */
uint8_t atk_mw8266d_get_ip(char *buf)
{
    uint8_t ret;
    char *p_start;
    char *p_end;

    ret = atk_mw8266d_send_at_cmd("AT+CIFSR", "OK", 500);
    if (ret != ATK_MW8266D_EOK)
    {
        return ATK_MW8266D_ERROR;
    }

    p_start = strstr((const char *)atk_mw8266d_uart_rx_get_frame(), "\"");
    p_end = strstr(p_start + 1, "\"");
    *p_end = '\0';
    sprintf(buf, "%s", p_start + 1);

    return ATK_MW8266D_EOK;
}

/**
 * @brief   ATK-MW8266D 连接 TCP 服务器
 * @param   server_ip  : TCP 服务器 IP 地址
 *          server_port: TCP 服务器端口号
 * @retval  ATK_MW8266D_EOK  : 连接 TCP 服务器成功
 *          ATK_MW8266D_ERROR: 连接 TCP 服务器失败
 */
uint8_t atk_mw8266d_connect_tcp_server(char *server_ip, char *server_port)
{
    uint8_t ret;
    char cmd[64];

    sprintf(cmd, "AT+CIPSTART=\"TCP\",\"%s\",%s", server_ip, server_port);
    ret = atk_mw8266d_send_at_cmd(cmd, "CONNECT", 5000);
    if (ret == ATK_MW8266D_EOK)
    {
        return ATK_MW8266D_EOK;
    }

    return ATK_MW8266D_ERROR;
}

/**
 * @brief   ATK-MW8266D 进入透传
 * @param   无
 * @retval  ATK_MW8266D_EOK  : 进入透传成功
 *          ATK_MW8266D_ERROR: 进入透传失败
 */
uint8_t atk_mw8266d_enter_unvarnished(void)
{
    uint8_t ret;

    ret  = atk_mw8266d_send_at_cmd("AT+CIPMODE=1", "OK", 10000);
    ret += atk_mw8266d_send_at_cmd("AT+CIPSEND", ">", 10000);
    if (ret == ATK_MW8266D_EOK)
    {
        return ATK_MW8266D_EOK;
    }

    return ATK_MW8266D_ERROR;
}

// ATK-MW8266D 退出透传
void atk_mw8266d_exit_unvarnished(void) { atk_mw8266d_uart_printf("+++"); }

/**
 * @brief   ATK-MW8266D 连接原子云服务器
 * @param   id : 原子云设备编号
 *          pwd: 原子云设备密码
 * @retval  ATK_MW8266D_EOK  : 连接原子云服务器成功
 *          ATK_MW8266D_ERROR: 连接原子云服务器失败
 */
uint8_t atk_mw8266d_connect_atkcld(char *id, char *pwd)
{
    uint8_t ret;
    char cmd[64];

    sprintf(cmd, "AT+ATKCLDSTA=\"%s\",\"%s\"", id, pwd);
    ret = atk_mw8266d_send_at_cmd(cmd, "CLOUD CONNECTED", 10000);
    if (ret == ATK_MW8266D_EOK)
    {
        return ATK_MW8266D_EOK;
    }
    else
    {
        return ATK_MW8266D_ERROR;
    }
}

/**
 * @brief   ATK-MW8266D 断开原子云服务器连接
 * @param   无
 * @retval  ATK_MW8266D_EOK  : 断开原子云服务器连接成功
 *          ATK_MW8266D_ERROR: 断开原子云服务器连接失败
 */
uint8_t atk_mw8266d_disconnect_atkcld(void)
{
    uint8_t ret;

    ret = atk_mw8266d_send_at_cmd("AT+ATKCLDCLS", "CLOUD DISCONNECT", 500);
    if (ret == ATK_MW8266D_EOK)
    {
        return ATK_MW8266D_EOK;
    }
    else
    {
        return ATK_MW8266D_ERROR;
    }
}
