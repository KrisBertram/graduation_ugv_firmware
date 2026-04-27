#include "isr_config.h"
#include "isr.h"
#include "wit_c_sdk.h"
#include "define.h"
#include "motion.h"
#include "hwt606.h"
#include "wifi.h"
#include "wifi_packet.h"
#include "gui.h"

// **************************** PIT 中断函数 ****************************
//xxx CCU60_CH0：暂无
IFX_INTERRUPT(cc60_pit_ch0_isr, CCU6_0_CH0_INT_VECTAB_NUM, CCU6_0_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    pit_clear_flag(CCU60_CH0);
}

//xxx CCU60_CH1：遥控器周期中断定时器 10us
IFX_INTERRUPT(cc60_pit_ch1_isr, CCU6_0_CH1_INT_VECTAB_NUM, CCU6_0_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    pit_clear_flag(CCU60_CH1);

    /* 遍历遥控器六个通道进行通道扫描 */
    x6fScan();
}

//xxx CCU61_CH1：电机、编码器周期中断定时器 5ms
IFX_INTERRUPT(cc61_pit_ch0_isr, CCU6_1_CH0_INT_VECTAB_NUM, CCU6_1_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    pit_clear_flag(CCU61_CH0);

    encoder_pulse = encoder_get_count(TIM_ENCODER); // 获取编码器计数
    encoder_clear_count(TIM_ENCODER);               // 清空编码器计数
    encoder_v = encoderSpeedCalculate(encoder_pulse);

    if(motor_duty >= 0)
    {
        pwm_set_duty(MOTOR_CH1, 0);                                             // 更新对应通道占空比
        pwm_set_duty(MOTOR_CH2, (uint32)(motor_duty * (PWM_DUTY_MAX / 100)));   // 更新对应通道占空比
    }
    else
    {
        pwm_set_duty(MOTOR_CH1, (uint32)(-motor_duty * (PWM_DUTY_MAX / 100)));  // 更新对应通道占空比
        pwm_set_duty(MOTOR_CH2, 0);                                             // 更新对应通道占空比
    }
}

//xxx CCU61_CH1：舵机、轮询周期中断定时器 2ms
IFX_INTERRUPT(cc61_pit_ch1_isr, CCU6_1_CH1_INT_VECTAB_NUM, CCU6_1_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    pit_clear_flag(CCU61_CH1);

    /* 轮询周期 */
    if (key_scan_flag < 255) ++key_scan_flag;
    if (atk_mw8266d_ready() && atk_mw8266d_info.isSerianet) { if (wifi_tx_timer < 255) ++wifi_tx_timer; }
    else { wifi_tx_timer = 0; }

    /* 更新舵机占空比 */
    pwm_set_duty(STEER_CH1, CLIP(steer_duty, STEER_DUTY_MIN, STEER_DUTY_MAX));
}
// **************************** PIT 中断函数 ****************************


// **************************** 串口中断函数 ****************************
//xxx UART0：WiFi 模块
IFX_INTERRUPT(uart0_tx_isr, UART0_INT_VECTAB_NUM, UART0_TX_INT_PRIO)
{
    interrupt_global_enable(0);     // 开启中断嵌套
}
IFX_INTERRUPT(uart0_rx_isr, UART0_INT_VECTAB_NUM, UART0_RX_INT_PRIO)
{
    interrupt_global_enable(0);     // 开启中断嵌套

#if DEBUG_UART_USE_INTERRUPT        // 如果开启 debug 串口中断
    debug_interrupr_handler();      // 调用 debug 串口接收处理函数 数据会被 debug 环形缓冲区读取
#endif                              // 如果修改了 DEBUG_UART_INDEX 那这段代码需要放到对应的串口中断去

    uint8 dat;
    uart_query_byte(ATK_MW8266D_UART, &dat);

    if (!atk_mw8266d_info.isSerianet || !atk_mw8266d_ready()) // 仅当 WiFi 没初始化完成的时候使用线性缓冲区，如果进入透传模式则直接使用字节流
    {
        // 判断 UART 接收缓冲是否溢出，留出一位给结束符 '\0'
        if (g_uart_rx_frame.sta.len < (ATK_MW8266D_UART_RX_BUF_SIZE - 1))
        {
            g_uart_rx_frame.buf[g_uart_rx_frame.sta.len] = dat; // 将接收到的数据写入缓冲
            g_uart_rx_frame.sta.len++;                          // 更新接收到的数据长度

            // 检查帧结束："\r\n"
            if (g_uart_rx_frame.sta.len >= 2)
            {
                // 判断倒数第二字节是 '\r'，当前字节是 '\n'
                if (g_uart_rx_frame.buf[g_uart_rx_frame.sta.len - 2] == '\r' &&
                    g_uart_rx_frame.buf[g_uart_rx_frame.sta.len - 1] == '\n')
                {
                    g_uart_rx_frame.sta.finsh = 1;     // 标记帧结束
                }
            }
        }
        // 如果 UART 接收缓冲溢出
        else
        {
            atk_mw8266d_uart_rx_restart();                      // 覆盖之前收到的数据
            g_uart_rx_frame.buf[g_uart_rx_frame.sta.len] = dat; // 将接收到的数据写入缓冲
            g_uart_rx_frame.sta.len++;                          // 更新接收到的数据长度
        }
    }
    else // TCP 服务连接成功，使用字节流解析数据包
    {
        serial_datapacket_recv(dat);
    }
}

//xxx UART3：HWT606 模块
IFX_INTERRUPT(uart3_tx_isr, UART3_INT_VECTAB_NUM, UART3_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
}
IFX_INTERRUPT(uart3_rx_isr, UART3_INT_VECTAB_NUM, UART3_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套

    if (hwtInitialized)
    {
        uint8 dat;
        while (uart_query_byte(HWT606_UART, &dat))
        {
            WitSerialDataIn(dat);
        }
    }
}

//xxx UART1：默认连接到摄像头配置串口
IFX_INTERRUPT(uart1_tx_isr, UART1_INT_VECTAB_NUM, UART1_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
}
IFX_INTERRUPT(uart1_rx_isr, UART1_INT_VECTAB_NUM, UART1_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    camera_uart_handler();                          // 摄像头参数配置统一回调函数
}

//xxx UART2：暂无
IFX_INTERRUPT(uart2_tx_isr, UART2_INT_VECTAB_NUM, UART2_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
}
IFX_INTERRUPT(uart2_rx_isr, UART2_INT_VECTAB_NUM, UART2_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    wireless_module_uart_handler();                 // 无线模块统一回调函数
}

//xxx UART4~11：暂无
IFX_INTERRUPT(uart4_tx_isr, UART4_INT_VECTAB_NUM, UART4_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
}
IFX_INTERRUPT(uart4_rx_isr, UART4_INT_VECTAB_NUM, UART4_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
}

IFX_INTERRUPT(uart5_tx_isr, UART5_INT_VECTAB_NUM, UART5_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
}
IFX_INTERRUPT(uart5_rx_isr, UART5_INT_VECTAB_NUM, UART5_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
}

IFX_INTERRUPT(uart6_tx_isr, UART6_INT_VECTAB_NUM, UART6_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
}
IFX_INTERRUPT(uart6_rx_isr, UART6_INT_VECTAB_NUM, UART6_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
}

IFX_INTERRUPT(uart8_tx_isr, UART8_INT_VECTAB_NUM, UART8_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
}
IFX_INTERRUPT(uart8_rx_isr, UART8_INT_VECTAB_NUM, UART8_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
}

IFX_INTERRUPT(uart9_tx_isr, UART9_INT_VECTAB_NUM, UART9_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
}
IFX_INTERRUPT(uart9_rx_isr, UART9_INT_VECTAB_NUM, UART9_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
}

IFX_INTERRUPT(uart10_tx_isr, UART10_INT_VECTAB_NUM, UART10_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
}
IFX_INTERRUPT(uart10_rx_isr, UART10_INT_VECTAB_NUM, UART10_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
}

IFX_INTERRUPT(uart11_tx_isr, UART11_INT_VECTAB_NUM, UART11_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
}
IFX_INTERRUPT(uart11_rx_isr, UART11_INT_VECTAB_NUM, UART11_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
}

//xxx 串口通讯错误中断
IFX_INTERRUPT(uart0_er_isr, UART0_INT_VECTAB_NUM, UART0_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart0_handle);
}
IFX_INTERRUPT(uart1_er_isr, UART1_INT_VECTAB_NUM, UART1_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart1_handle);
}
IFX_INTERRUPT(uart2_er_isr, UART2_INT_VECTAB_NUM, UART2_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart2_handle);
}
IFX_INTERRUPT(uart3_er_isr, UART3_INT_VECTAB_NUM, UART3_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart3_handle);
}
IFX_INTERRUPT(uart4_er_isr, UART4_INT_VECTAB_NUM, UART4_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart4_handle);
}
IFX_INTERRUPT(uart5_er_isr, UART5_INT_VECTAB_NUM, UART5_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart5_handle);
}
IFX_INTERRUPT(uart6_er_isr, UART6_INT_VECTAB_NUM, UART6_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart6_handle);
}
IFX_INTERRUPT(uart8_er_isr, UART8_INT_VECTAB_NUM, UART8_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart8_handle);
}
IFX_INTERRUPT(uart9_er_isr, UART9_INT_VECTAB_NUM, UART9_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart9_handle);
}
IFX_INTERRUPT(uart10_er_isr, UART10_INT_VECTAB_NUM, UART10_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart10_handle);
}
IFX_INTERRUPT(uart11_er_isr, UART11_INT_VECTAB_NUM, UART11_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart11_handle);
}
// **************************** 串口中断函数 ****************************


// **************************** 外部中断函数 ****************************
IFX_INTERRUPT(exti_ch0_ch4_isr, EXTI_CH0_CH4_INT_VECTAB_NUM, EXTI_CH0_CH4_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    if(exti_flag_get(ERU_CH0_REQ0_P15_4))           // 通道 0 中断
    {
        exti_flag_clear(ERU_CH0_REQ0_P15_4);
    }

    if(exti_flag_get(ERU_CH4_REQ13_P15_5))          // 通道 4 中断
    {
        exti_flag_clear(ERU_CH4_REQ13_P15_5);
    }
}

IFX_INTERRUPT(exti_ch1_ch5_isr, EXTI_CH1_CH5_INT_VECTAB_NUM, EXTI_CH1_CH5_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套

    if(exti_flag_get(ERU_CH1_REQ10_P14_3))          // 通道 1 中断
    {
        exti_flag_clear(ERU_CH1_REQ10_P14_3);
        tof_module_exti_handler();                  // ToF 模块 INT 更新中断
    }

    if(exti_flag_get(ERU_CH5_REQ1_P15_8))           // 通道 5 中断
    {
        exti_flag_clear(ERU_CH5_REQ1_P15_8);
    }
}

// 由于摄像头 pclk 引脚默认占用了 2 通道，用于触发 DMA，因此这里不再定义中断函数
// IFX_INTERRUPT(exti_ch2_ch6_isr, EXTI_CH2_CH6_INT_VECTAB_NUM, EXTI_CH2_CH6_INT_PRIO)
// {
//  interrupt_global_enable(0);                     // 开启中断嵌套
//  if(exti_flag_get(ERU_CH2_REQ7_P00_4))           // 通道2中断
//  {
//      exti_flag_clear(ERU_CH2_REQ7_P00_4);
//  }
//  if(exti_flag_get(ERU_CH6_REQ9_P20_0))           // 通道6中断
//  {
//      exti_flag_clear(ERU_CH6_REQ9_P20_0);
//  }
// }

IFX_INTERRUPT(exti_ch3_ch7_isr, EXTI_CH3_CH7_INT_VECTAB_NUM, EXTI_CH3_CH7_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    if(exti_flag_get(ERU_CH3_REQ6_P02_0))           // 通道 3 中断
    {
        exti_flag_clear(ERU_CH3_REQ6_P02_0);
        camera_vsync_handler();                     // 摄像头触发采集统一回调函数
    }
    if(exti_flag_get(ERU_CH7_REQ16_P15_1))          // 通道 7 中断
    {
        exti_flag_clear(ERU_CH7_REQ16_P15_1);

    }
}
// **************************** 外部中断函数 ****************************


// **************************** DMA 中断函数 ****************************
IFX_INTERRUPT(dma_ch5_isr, DMA_INT_VECTAB_NUM, DMA_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    camera_dma_handler();                           // 摄像头采集完成统一回调函数
}
// **************************** DMA 中断函数 ****************************
