#ifndef CODE_PATH_FOLLOW_H_
#define CODE_PATH_FOLLOW_H_

#include "zf_common_headfile.h"
#include "define.h"
#include "ins.h"
#include "trajectory.h"

/*
 * PATH_START_MAX_DIST_M：
 *   启动自动跟踪时，小车当前位置到轨迹起点允许的最大距离，单位 m。
 *   调大后更容易从偏离位置启动，但可能在场地中间突然冲向轨迹；调小更安全但摆车要求更准。
 *   3m x 6m 场地推荐 0.30~0.70m，默认 0.50m。
 */
#ifndef PATH_START_MAX_DIST_M
#define PATH_START_MAX_DIST_M           (0.50f)
#endif

/*
 * PATH_LOST_MAX_DIST_M：
 *   最近轨迹点横向距离超过该值时判定丢线并停车，单位 m。
 *   调大后容错更强但越界风险更高；调小更安全但可能因定位误差频繁停车。
 *   3m x 6m 场地推荐 0.50~1.00m，默认 0.80m。
 */
#ifndef PATH_LOST_MAX_DIST_M
#define PATH_LOST_MAX_DIST_M            (0.80f)
#endif

/*
 * PATH_FINISH_DIST_M：
 *   距离轨迹终点小于该值时停车，单位 m。
 *   调大后更早停车；调小会更贴近终点但可能越过终点。
 *   推荐 0.08~0.20m，默认 0.12m。
 */
#ifndef PATH_FINISH_DIST_M
#define PATH_FINISH_DIST_M              (0.12f)
#endif

/*
 * PATH_LOOKAHEAD_BASE_M / PATH_LOOKAHEAD_GAIN / MIN / MAX：
 *   Pure Pursuit 预瞄距离 Ld = base + gain * speed，并进行限幅，单位 m。
 *   预瞄调大后轨迹更平稳但弯道会切弯、响应变钝；调小后更贴线但容易左右振荡。
 *   小场地低速推荐 Ld 约 0.30~0.65m。
 */
#ifndef PATH_LOOKAHEAD_BASE_M
#define PATH_LOOKAHEAD_BASE_M           (0.35f)
#endif
#ifndef PATH_LOOKAHEAD_GAIN
#define PATH_LOOKAHEAD_GAIN             (0.50f)
#endif
#ifndef PATH_LOOKAHEAD_MIN_M
#define PATH_LOOKAHEAD_MIN_M            (0.30f)
#endif
#ifndef PATH_LOOKAHEAD_MAX_M
#define PATH_LOOKAHEAD_MAX_M            (0.65f)
#endif

/*
 * PATH_PP_DUTY_PER_CURVATURE：
 *   Pure Pursuit 曲率前馈到舵机 duty 偏移的换算增益，单位 duty/(1/m)。
 *   调大后弯道打舵更积极，可能更贴线也可能过冲；调小后转弯更保守，可能外抛。
 *   推荐 80~260，默认 140。
 */
#ifndef PATH_PP_DUTY_PER_CURVATURE
#define PATH_PP_DUTY_PER_CURVATURE      (140.0f)
#endif

/*
 * PATH_LAT_KP_DUTY_PER_M：
 *   横向误差 P 增益，单位 duty/m。e_y > 0 表示车在轨迹左侧，本项会减小 duty 使车向右修正。
 *   调大后回到轨迹更快，但过大会左右摆；调小更稳但纠偏慢。
 *   推荐 80~400，默认 180。
 */
#ifndef PATH_LAT_KP_DUTY_PER_M
#define PATH_LAT_KP_DUTY_PER_M          (180.0f)
#endif

/*
 * PATH_YAW_KP_DUTY_PER_RAD：
 *   航向误差 P 增益，单位 duty/rad。e_yaw > 0 表示车头相对轨迹切线偏左，本项会减小 duty。
 *   调大后车头更快对齐轨迹，但过大会让舵机来回抖；调小则车头对齐慢。
 *   推荐 50~250，默认 120。
 */
#ifndef PATH_YAW_KP_DUTY_PER_RAD
#define PATH_YAW_KP_DUTY_PER_RAD        (120.0f)
#endif

/*
 * PATH_LAT_KI_DUTY_PER_M_S：
 *   横向误差 I 增益，单位 duty/(m*s)，用于补偿舵机中位偏差、左右机械差异等长期偏差。
 *   调大后能更快消除长期偏差，但过大会积分过冲；调小则补偿慢。
 *   推荐 0~80，默认 20。若调试初期左右摆明显，可先设为 0。
 */
#ifndef PATH_LAT_KI_DUTY_PER_M_S
#define PATH_LAT_KI_DUTY_PER_M_S        (20.0f)
#endif

/*
 * PATH_LAT_KD_DUTY_PER_MPS：
 *   横向误差 D 增益，单位 duty/(m/s)，对横向误差变化率进行阻尼。
 *   默认 0 表示不启用 D 项；后续如果车身明显滞后且 e_y 噪声不大，可从 10~30 小幅尝试。
 *   调大可抑制过冲，但过大会放大定位噪声导致舵机抖动。
 */
#ifndef PATH_LAT_KD_DUTY_PER_MPS
#define PATH_LAT_KD_DUTY_PER_MPS        (0.0f)
#endif

/*
 * PATH_LAT_INTEGRAL_LIMIT_M_S：
 *   横向误差积分限幅，单位 m*s，用于防止 I 项长时间累积后把舵机推到一侧。
 *   调大补偿能力更强但风险更高；调小更安全但可能消不掉中位偏差。
 *   推荐 0.20~1.00，默认 0.50。
 */
#ifndef PATH_LAT_INTEGRAL_LIMIT_M_S
#define PATH_LAT_INTEGRAL_LIMIT_M_S     (0.50f)
#endif

/*
 * PATH_INTEGRAL_MIN_SPEED_MPS：
 *   只有车速高于该阈值时才累计横向积分，单位 m/s。
 *   调大可避免低速原地抖动时积分乱飘；调小则低速也会补偿偏差。
 *   推荐 0.02~0.08m/s，默认 0.04m/s。
 */
#ifndef PATH_INTEGRAL_MIN_SPEED_MPS
#define PATH_INTEGRAL_MIN_SPEED_MPS     (0.04f)
#endif

/*
 * PATH_STEER_SLEW_DUTY_PER_S：
 *   舵机 duty 命令变化率限制，单位 duty/s。
 *   调大响应更快但动作更猛；调小更柔和但可能跟不上急弯。
 *   推荐 500~1800，默认 900。
 */
#ifndef PATH_STEER_SLEW_DUTY_PER_S
#define PATH_STEER_SLEW_DUTY_PER_S      (900.0f)
#endif

/*
 * PATH_V_MAX_MPS：
 *   自动跟踪最高目标速度，单位 m/s。
 *   调大能更快完成轨迹，但对定位、舵机和电机响应要求更高；调小更安全。
 *   首次上板推荐 0.20~0.30m/s，默认 0.30m/s。
 */
#ifndef PATH_V_MAX_MPS
#define PATH_V_MAX_MPS                  (0.30f)
#endif

/*
 * PATH_A_LAT_MAX_MPS2：
 *   曲率限速使用的横向加速度上限，单位 m/s^2。
 *   调小后弯道自动更慢更安全；调大后弯道速度更高但更容易外抛。
 *   推荐 0.08~0.20，默认 0.12。
 */
#ifndef PATH_A_LAT_MAX_MPS2
#define PATH_A_LAT_MAX_MPS2             (0.12f)
#endif

/*
 * PATH_END_SLOW_DIST_M：
 *   终点前线性减速距离，单位 m。
 *   调大后更早减速、更稳；调小后终点前速度保持更久，可能越过终点。
 *   小场地推荐 0.50~1.00m，默认 0.70m。
 */
#ifndef PATH_END_SLOW_DIST_M
#define PATH_END_SLOW_DIST_M            (0.70f)
#endif

/*
 * PATH_SPEED_KP_DUTY_PER_MPS / PATH_SPEED_KI_DUTY_PER_M：
 *   速度 PI 增益。P 负责即时提速，I 负责补偿地面阻力和电机死区。
 *   P 调大响应快但可能冲，I 调大能克服死区但可能越积越大。
 *   默认 P=35、I=12；首次上板如车不起步，优先小幅增大 I 或电机 duty 上限。
 */
#ifndef PATH_SPEED_KP_DUTY_PER_MPS
#define PATH_SPEED_KP_DUTY_PER_MPS      (35.0f)
#endif
#ifndef PATH_SPEED_KI_DUTY_PER_M
#define PATH_SPEED_KI_DUTY_PER_M        (12.0f)
#endif

/*
 * PATH_SPEED_INTEGRAL_LIMIT_M：
 *   速度误差积分限幅，单位 m，防止电机 duty 因长时间误差累积过大。
 *   推荐 0.10~0.60，默认 0.30。
 */
#ifndef PATH_SPEED_INTEGRAL_LIMIT_M
#define PATH_SPEED_INTEGRAL_LIMIT_M     (0.30f)
#endif

/*
 * PATH_MOTOR_DUTY_MAX：
 *   自动模式电机最大占空比百分比。调大速度上限和加速能力更强，但小场地风险更高。
 *   首次上板推荐 8~14，默认 14；确认安全后再逐步增加。
 */
#ifndef PATH_MOTOR_DUTY_MAX
#define PATH_MOTOR_DUTY_MAX             (14.0f)
#endif

typedef enum {
    PATH_FOLLOW_IDLE = 0,
    PATH_FOLLOW_RUNNING,
    PATH_FOLLOW_FINISHED,
    PATH_FOLLOW_ERROR
} PathFollowStatus_t;

typedef struct {
    uint8 selected_index;       // 当前选中的预置轨迹编号
    uint8 active_index;         // 当前正在运行或最近一次生成的轨迹编号
    uint8 status;               // PathFollowStatus_t

    uint16 nearest_index;       // 最近轨迹点索引
    uint16 target_index;        // 预瞄目标点索引

    float e_y;                  // 横向误差，单位 m，左正右负
    float e_yaw;                // 航向误差，单位 rad，左偏为正
    float e_y_derivative;       // 横向误差变化率，单位 m/s
    float lat_integral;         // 横向误差积分，单位 m*s

    float lookahead;            // 当前预瞄距离，单位 m
    float kappa_pp;             // Pure Pursuit 曲率前馈，单位 1/m
    float steer_offset;         // 相对 STEER_DUTY_M 的 duty 偏移
    float steer_duty_cmd;       // 最终舵机 duty 命令

    float v_ref;                // 当前目标速度，单位 m/s
    float speed_error;          // 速度误差，单位 m/s
    float speed_integral;       // 速度误差积分，单位 m
    float remaining_s;          // 剩余路径长度，单位 m
} PathFollowerState_t;

extern volatile PathFollowerState_t pathFollower;

uint8 pathFollowerSelectPrev(void);
uint8 pathFollowerSelectNext(void);
uint8 pathFollowerStart(const volatile CarPoseState *pose);
void pathFollowerStop(void);
void pathFollowerUpdate(const volatile CarPoseState *pose, float dt, uint8 rc_enable);
uint8 pathFollowerIsRunning(void);

#endif /* CODE_PATH_FOLLOW_H_ */
