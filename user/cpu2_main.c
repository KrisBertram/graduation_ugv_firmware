#include "zf_common_headfile.h"
#include "define.h"
#include "motion.h"
#include "mmath.h"
#include "hwt606.h"
#include "wifi.h"
#include "wifi_packet.h"
#include "ins.h"
#include "gui.h"
#include "path_follow.h"
#pragma section all "cpu2_dsram"

void core2_main(void)
{
    disable_Watchdog();         // 关闭看门狗
    interrupt_global_enable(0); // 打开全局中断

    /* 屏幕初始化 */
    ips200_set_dir(IPS200_PORTAIT);
    ips200_set_color(RGB565_CYAN, RGB565_BLACK);
    ips200_init(IPS200_TYPE_PARALLEL8);

    cpu_wait_event_ready();     // 等待所有核心初始化完毕

    while (TRUE)
    {
//        ips200_show_string(0, 0*16, "rxflag");
//        ips200_show_float(80, 0*16, (float)g_uart_rx_frame.sta.finsh, 8, 6);
//
//        uint16 y = 0; y += 16;
//
//        ips200_show_string(0, y, "PowerOn:");
//        ips200_show_float(80, y, atk_mw8266d_info.isPowerOn, 8, 6);
//        y += 16;
//
//        ips200_show_string(0, y, "ModeSet:");
//        ips200_show_float(80, y, atk_mw8266d_info.isModeSet, 8, 6);
//        y += 16;
//
//        ips200_show_string(0, y, "WiFiConn:");
//        ips200_show_float(80, y, atk_mw8266d_info.isWiFiConnected, 8, 6);
//        y += 16;
//
//        ips200_show_string(0, y, "TCPConn:");
//        ips200_show_float(80, y, atk_mw8266d_info.isTcpConnected, 8, 6);
//        y += 16;
//
//        ips200_show_string(0, y, "Serianet:");
//        ips200_show_float(80, y, atk_mw8266d_info.isSerianet, 8, 6);
//        y += 16;

//        steer_duty = CLIP(steer_duty, STEER_DUTY_MIN, STEER_DUTY_MAX);
//
//        ips200_show_string(0, 1*16, "steer");
//        ips200_show_float(80, 1*16, (float)steer_duty, 8, 6);
//
//        ips200_show_string(0, 2*16, "x6fch1");
//        ips200_show_float(80, 2*16, (float)x6f_channel[0].out, 8, 6);
//
//        ips200_show_string(0, 3*16, "x6fch2");
//        ips200_show_float(80, 3*16, (float)x6f_channel[1].out, 8, 6);
//
//        ips200_show_string(0, 4*16, "x6fch3");
//        ips200_show_float(80, 4*16, (float)x6f_channel[2].out, 8, 6);
//
//        ips200_show_string(0, 5*16, "x6fch4");
//        ips200_show_float(80, 5*16, (float)x6f_channel[3].out, 8, 6);
//
//        ips200_show_string(0, 6*16, "keyflag");
//        ips200_show_float(80, 6*16, (float)key_scan_flag, 8, 6);
//
//        ips200_show_string(0, 7*16, "rcflag");
//        ips200_show_float(80, 7*16, (float)rc_enable_flag, 8, 6);
//
//        ips200_show_string(0, 8*16, "servo");
//        ips200_show_float(80, 8*16, (float)steer_duty, 8, 6);
//
//        ips200_show_string(0, 9*16, "motor");
//        ips200_show_float(80, 9*16, (float)motor_duty, 8, 6);
//
//        ips200_show_string(0, 10*16, "encoder");
//        ips200_show_float(80, 10*16, (float)encoder_pulse, 8, 6);


        uint16 y = 0;

        //--------- 轨迹跟踪调试 ---------
        ips200_show_string(0, y, "traj");   ips200_show_uint(60, y, (uint32)pathFollower.selected_index, 4); y+=16;
        ips200_show_string(0, y, "status"); ips200_show_uint(60, y, (uint32)pathFollower.status, 4);         y+=16;
        ips200_show_string(0, y, "near");   ips200_show_uint(60, y, (uint32)pathFollower.nearest_index, 6);  y+=16;
        ips200_show_string(0, y, "target"); ips200_show_uint(60, y, (uint32)pathFollower.target_index, 6);   y+=16;
        ips200_show_string(0, y, "e_y");    ips200_show_float(60, y, pathFollower.e_y, 8, 4);                   y+=16;
        ips200_show_string(0, y, "e_yaw");  ips200_show_float(60, y, pathFollower.e_yaw, 8, 4);                 y+=16;
        ips200_show_string(0, y, "v_ref");  ips200_show_float(60, y, pathFollower.v_ref, 8, 4);                 y+=16;
        ips200_show_string(0, y, "remain"); ips200_show_float(60, y, pathFollower.remaining_s, 8, 4);           y+=16;
        ips200_show_string(0, y, "motor");  ips200_show_float(60, y, motor_duty, 8, 3);                         y+=16;
        ips200_show_string(0, y, "steer");  ips200_show_float(60, y, steer_duty, 8, 2);                         y+=16;

        //--------- 航位解算 ---------
        ips200_show_string(0, y, "odom x");   ips200_show_float(80, y, carPose.x, 8, 4);        y+=16;
        ips200_show_string(0, y, "odom y");   ips200_show_float(80, y, carPose.y, 8, 4);        y+=16;
        ips200_show_string(0, y, "odom yaw"); ips200_show_float(80, y, carPose.yaw_deg, 8, 3);  y+=16;
        ips200_show_string(0, y, "speed");    ips200_show_float(80, y, encoder_v, 8, 4);        y+=16;
        ips200_show_string(0, y, "imu yaw");  ips200_show_float(80, y, imu_dat.yaw, 8, 3);      y+=16;

//        //--------- 四元数 ---------
//        ips200_show_string(0, y, "Q0");  ips200_show_float(60, y, imu_dat.Q0, 8, 4);  y+=16;
//        ips200_show_string(0, y, "Q1");  ips200_show_float(60, y, imu_dat.Q1, 8, 4);  y+=16;
//        ips200_show_string(0, y, "Q2");  ips200_show_float(60, y, imu_dat.Q2, 8, 4);  y+=16;
//        ips200_show_string(0, y, "Q3");  ips200_show_float(60, y, imu_dat.Q3, 8, 4);  y+=16;

        //--------- 时间 ---------
//        char buf[32];
//        snprintf(buf, sizeof(buf), "%d-%d-%d", imu_dat.yy, imu_dat.mm, imu_dat.dd);
//        ips200_show_string(0, y, "Date");  ips200_show_string(60, y, buf);  y+=16;
//
//        snprintf(buf, sizeof(buf), "%d:%d:%d.%d", imu_dat.hh, imu_dat.mn, imu_dat.ss, imu_dat.ms);
//        ips200_show_string(0, y, "Time");  ips200_show_string(60, y, buf);  y+=16;

//        /* 下行数据包数据 */
//        uint16 y = 0;
//
//        ips200_show_string(0, y, "speed:");
//        ips200_show_float(80, y, recv_packet.speed, 8, 6);
//        y += 16;
//
//        ips200_show_string(0, y, "yaw:");
//        ips200_show_float(80, y, recv_packet.yaw, 8, 6);
//        y += 16;
//
//        ips200_show_string(0, y, "pitch:");
//        ips200_show_float(80, y, recv_packet.pitch, 8, 6);
//        y += 16;
//
//        ips200_show_string(0, y, "distance:");
//        ips200_show_float(80, y, recv_packet.distance, 8, 6);
//        y += 16;
//
//        ips200_show_string(0, y, "action:");
//        ips200_show_float(80, y, recv_packet.action, 8, 6);
//        y += 16;
    }
}

#pragma section all restore /* section all "cpu2_dsram" 与 section all restore 语句之间的全局变量都放在 CPU2 的 RAM 中 */
