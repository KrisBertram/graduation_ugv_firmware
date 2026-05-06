#include "zf_common_headfile.h"
#include "define.h"
#include "mmath.h"
#include "motion.h"
#include "hwt606.h"
#include "wifi.h"
#include "wifi_packet.h"
#include "ins.h"
#include "gui.h"
#include "trajectory.h"
#include "path_follow.h"
#include "menu_gui.h"

#define MENU_COLS                       (30u)
#define MENU_ROWS                       (20u)
#define MENU_BODY_FIRST_ROW             (1u)
#define MENU_BODY_LAST_ROW              (18u)
#define MENU_FOOTER_ROW                 (19u)
#define MENU_ROW_H                      (16u)
#define MENU_VALUE_COL                  (15u)
#define MENU_LABEL_COLS                 (14u)
#define MENU_REFRESH_TICKS              (50u)   // 按键扫描 4ms 一次，约 200ms 小范围刷新
#define MENU_MESSAGE_TICKS              (180u)
#define MENU_NUMERIC_BUF_LEN            (14u)

#define MENU_COLOR_BG                   (RGB565_BLACK)
#define MENU_COLOR_FG                   (RGB565_WHITE)
#define MENU_COLOR_DIM                  (RGB565_GRAY)
#define MENU_COLOR_HEADER               (RGB565_CYAN)
#define MENU_COLOR_SELECT_FG            (RGB565_BLACK)
#define MENU_COLOR_SELECT_BG            (RGB565_CYAN)
#define MENU_COLOR_EDIT_BG              (RGB565_YELLOW)
#define MENU_COLOR_ERROR                (RGB565_RED)
#define MENU_COLOR_OK                   (RGB565_GREEN)
#define MENU_COLOR_PATH                 (RGB565_YELLOW)
#define MENU_COLOR_AXIS                 (RGB565_CYAN)

typedef enum {
    MENU_PAGE_MAIN = 0,
    MENU_PAGE_PARAMS,
    MENU_PAGE_TUNE,
    MENU_PAGE_TRAJECTORY,
    MENU_PAGE_SYSTEM,
    MENU_PAGE_PREVIEW
} MenuPage_t;

typedef enum {
    MENU_EDIT_NONE = 0,
    MENU_EDIT_STEP,
    MENU_EDIT_NUMERIC
} MenuEditMode_t;

typedef enum {
    PARAM_CAT_ODOM = 0,
    PARAM_CAT_IMU,
    PARAM_CAT_MOTION,
    PARAM_CAT_PATH,
    PARAM_CAT_WIFI,
    PARAM_CAT_COUNT
} ParamCategory_t;

typedef enum {
    TUNE_CAT_SAFETY = 0,
    TUNE_CAT_LOOKAHEAD,
    TUNE_CAT_CONTROL,
    TUNE_CAT_SPEED,
    TUNE_CAT_COUNT
} TuneCategory_t;

typedef enum {
    MVAL_FLOAT = 0,
    MVAL_INT,
    MVAL_UINT,
    MVAL_BOOL,
    MVAL_TEXT
} MenuValueType_t;

typedef enum {
    VID_CAR_X = 0,
    VID_CAR_Y,
    VID_CAR_YAW_DEG,
    VID_CAR_YAW_RAD,
    VID_CAR_VX,
    VID_CAR_VY,
    VID_CAR_SPEED,
    VID_CAR_DISTANCE,
    VID_CAR_YAW_RATE,
    VID_CAR_STEER_ANGLE,
    VID_CAR_GYRO_BIAS,
    VID_CAR_READY,

    VID_IMU_ROLL,
    VID_IMU_PITCH,
    VID_IMU_YAW,
    VID_IMU_GX,
    VID_IMU_GY,
    VID_IMU_GZ,
    VID_IMU_AX,
    VID_IMU_AY,
    VID_IMU_AZ,
    VID_IMU_Q0,
    VID_IMU_Q1,
    VID_IMU_Q2,
    VID_IMU_Q3,
    VID_IMU_HOUR,
    VID_IMU_MIN,
    VID_IMU_SEC,
    VID_IMU_MS,

    VID_ENCODER_PULSE,
    VID_ENCODER_V,
    VID_MOTOR_DUTY,
    VID_STEER_DUTY,
    VID_RC_ENABLE,
    VID_X6F_CH1,
    VID_X6F_CH2,
    VID_X6F_CH3,
    VID_X6F_CH4,
    VID_X6F_CH5,
    VID_X6F_CH6,

    VID_PATH_SELECTED,
    VID_PATH_ACTIVE,
    VID_PATH_STATUS,
    VID_PATH_NEAREST,
    VID_PATH_TARGET,
    VID_PATH_EY,
    VID_PATH_EYAW,
    VID_PATH_EY_D,
    VID_PATH_LAT_I,
    VID_PATH_LOOKAHEAD,
    VID_PATH_KAPPA,
    VID_PATH_STEER_OFFSET,
    VID_PATH_STEER_CMD,
    VID_PATH_VREF,
    VID_PATH_SPEED_ERR,
    VID_PATH_SPEED_I,
    VID_PATH_REMAIN,

    VID_WIFI_POWER,
    VID_WIFI_MODE,
    VID_WIFI_CONN,
    VID_TCP_CONN,
    VID_SERIANET,
    VID_RX_FINISH,
    VID_RX_LEN,
    VID_RECV_SPEED,
    VID_RECV_YAW,
    VID_RECV_PITCH,
    VID_RECV_ROLL,
    VID_RECV_DIST,
    VID_RECV_ACTION
} MenuValueId_t;

typedef struct {
    const char *name;
    MenuValueType_t type;
    MenuValueId_t id;
    uint8 decimals;
} ParamItem_t;

typedef enum {
    TUNE_RULE_NONE = 0,
    TUNE_RULE_LOOKAHEAD_MIN,
    TUNE_RULE_LOOKAHEAD_MAX
} TuneRule_t;

typedef struct {
    const char *name;
    volatile float *value;
    float min_value;
    float max_value;
    float step;
    uint8 decimals;
    TuneRule_t rule;
} TuneItem_t;

typedef struct {
    const char *name;
    const ParamItem_t *items;
    uint8 count;
} ParamCategoryInfo_t;

typedef struct {
    const char *name;
    const TuneItem_t *items;
    uint8 count;
} TuneCategoryInfo_t;

static const char *main_items[] = {
    "Params",
    "Tune",
    "Trajectory",
    "System"
};

static const ParamItem_t params_odom[] = {
    { "x m",             MVAL_FLOAT, VID_CAR_X,           3 },
    { "y m",             MVAL_FLOAT, VID_CAR_Y,           3 },
    { "yaw deg",         MVAL_FLOAT, VID_CAR_YAW_DEG,     2 },
    { "yaw rad",         MVAL_FLOAT, VID_CAR_YAW_RAD,     3 },
    { "vx mps",          MVAL_FLOAT, VID_CAR_VX,          3 },
    { "vy mps",          MVAL_FLOAT, VID_CAR_VY,          3 },
    { "speed mps",       MVAL_FLOAT, VID_CAR_SPEED,       3 },
    { "distance m",      MVAL_FLOAT, VID_CAR_DISTANCE,    3 },
    { "yaw rate",        MVAL_FLOAT, VID_CAR_YAW_RATE,    3 },
    { "steer angle",     MVAL_FLOAT, VID_CAR_STEER_ANGLE, 3 },
    { "gyro bias",       MVAL_FLOAT, VID_CAR_GYRO_BIAS,   4 },
    { "pose ready",      MVAL_BOOL,  VID_CAR_READY,       0 }
};

static const ParamItem_t params_imu[] = {
    { "roll deg",        MVAL_FLOAT, VID_IMU_ROLL,        2 },
    { "pitch deg",       MVAL_FLOAT, VID_IMU_PITCH,       2 },
    { "yaw deg",         MVAL_FLOAT, VID_IMU_YAW,         2 },
    { "gx dps",          MVAL_FLOAT, VID_IMU_GX,          2 },
    { "gy dps",          MVAL_FLOAT, VID_IMU_GY,          2 },
    { "gz dps",          MVAL_FLOAT, VID_IMU_GZ,          2 },
    { "ax g",            MVAL_FLOAT, VID_IMU_AX,          3 },
    { "ay g",            MVAL_FLOAT, VID_IMU_AY,          3 },
    { "az g",            MVAL_FLOAT, VID_IMU_AZ,          3 },
    { "Q0",              MVAL_FLOAT, VID_IMU_Q0,          4 },
    { "Q1",              MVAL_FLOAT, VID_IMU_Q1,          4 },
    { "Q2",              MVAL_FLOAT, VID_IMU_Q2,          4 },
    { "Q3",              MVAL_FLOAT, VID_IMU_Q3,          4 },
    { "hour",            MVAL_UINT,  VID_IMU_HOUR,        0 },
    { "minute",          MVAL_UINT,  VID_IMU_MIN,         0 },
    { "second",          MVAL_UINT,  VID_IMU_SEC,         0 },
    { "ms",              MVAL_UINT,  VID_IMU_MS,          0 }
};

static const ParamItem_t params_motion[] = {
    { "encoder pulse",   MVAL_INT,   VID_ENCODER_PULSE,  0 },
    { "encoder v",       MVAL_FLOAT, VID_ENCODER_V,      3 },
    { "motor duty",      MVAL_FLOAT, VID_MOTOR_DUTY,     2 },
    { "steer duty",      MVAL_FLOAT, VID_STEER_DUTY,     1 },
    { "rc enable",       MVAL_BOOL,  VID_RC_ENABLE,      0 },
    { "x6f ch1",         MVAL_INT,   VID_X6F_CH1,        0 },
    { "x6f ch2",         MVAL_INT,   VID_X6F_CH2,        0 },
    { "x6f ch3",         MVAL_INT,   VID_X6F_CH3,        0 },
    { "x6f ch4",         MVAL_INT,   VID_X6F_CH4,        0 },
    { "x6f ch5",         MVAL_INT,   VID_X6F_CH5,        0 },
    { "x6f ch6",         MVAL_INT,   VID_X6F_CH6,        0 }
};

static const ParamItem_t params_path[] = {
    { "selected",        MVAL_UINT,  VID_PATH_SELECTED,     0 },
    { "active",          MVAL_UINT,  VID_PATH_ACTIVE,       0 },
    { "status",          MVAL_TEXT,  VID_PATH_STATUS,       0 },
    { "nearest",         MVAL_UINT,  VID_PATH_NEAREST,      0 },
    { "target",          MVAL_UINT,  VID_PATH_TARGET,       0 },
    { "e_y m",           MVAL_FLOAT, VID_PATH_EY,           3 },
    { "e_yaw rad",       MVAL_FLOAT, VID_PATH_EYAW,         3 },
    { "e_y der",         MVAL_FLOAT, VID_PATH_EY_D,         3 },
    { "lat integral",    MVAL_FLOAT, VID_PATH_LAT_I,        3 },
    { "lookahead",       MVAL_FLOAT, VID_PATH_LOOKAHEAD,    3 },
    { "kappa pp",        MVAL_FLOAT, VID_PATH_KAPPA,        3 },
    { "steer offset",    MVAL_FLOAT, VID_PATH_STEER_OFFSET, 2 },
    { "steer cmd",       MVAL_FLOAT, VID_PATH_STEER_CMD,    1 },
    { "v ref",           MVAL_FLOAT, VID_PATH_VREF,         3 },
    { "speed err",       MVAL_FLOAT, VID_PATH_SPEED_ERR,    3 },
    { "speed int",       MVAL_FLOAT, VID_PATH_SPEED_I,      3 },
    { "remain m",        MVAL_FLOAT, VID_PATH_REMAIN,       3 }
};

static const ParamItem_t params_wifi[] = {
    { "power",           MVAL_BOOL,  VID_WIFI_POWER,     0 },
    { "mode set",        MVAL_BOOL,  VID_WIFI_MODE,      0 },
    { "wifi conn",       MVAL_BOOL,  VID_WIFI_CONN,      0 },
    { "tcp conn",        MVAL_BOOL,  VID_TCP_CONN,       0 },
    { "transparent",     MVAL_BOOL,  VID_SERIANET,       0 },
    { "rx finish",       MVAL_BOOL,  VID_RX_FINISH,      0 },
    { "rx len",          MVAL_UINT,  VID_RX_LEN,         0 },
    { "recv speed",      MVAL_FLOAT, VID_RECV_SPEED,     3 },
    { "recv yaw",        MVAL_FLOAT, VID_RECV_YAW,       2 },
    { "recv pitch",      MVAL_FLOAT, VID_RECV_PITCH,     2 },
    { "recv roll",       MVAL_FLOAT, VID_RECV_ROLL,      2 },
    { "recv dist",       MVAL_FLOAT, VID_RECV_DIST,      3 },
    { "recv action",     MVAL_UINT,  VID_RECV_ACTION,    0 }
};

static const ParamCategoryInfo_t param_categories[PARAM_CAT_COUNT] = {
    { "Odom/INS",  params_odom,   (uint8)ARRAY_SIZE(params_odom)   },
    { "IMU",       params_imu,    (uint8)ARRAY_SIZE(params_imu)    },
    { "Motion/RC", params_motion, (uint8)ARRAY_SIZE(params_motion) },
    { "Path",      params_path,   (uint8)ARRAY_SIZE(params_path)   },
    { "WiFi/RX",   params_wifi,   (uint8)ARRAY_SIZE(params_wifi)   }
};

static const TuneItem_t tune_safety[] = {
    { "start max m",  &pathFollowConfig.start_max_dist_m,  0.05f, 1.50f, 0.05f, 2, TUNE_RULE_NONE },
    { "lost max m",   &pathFollowConfig.lost_max_dist_m,   0.10f, 2.00f, 0.05f, 2, TUNE_RULE_NONE },
    { "finish m",     &pathFollowConfig.finish_dist_m,     0.02f, 0.50f, 0.01f, 2, TUNE_RULE_NONE }
};

static const TuneItem_t tune_lookahead[] = {
    { "base m",       &pathFollowConfig.lookahead_base_m,  0.05f, 1.20f, 0.01f, 2, TUNE_RULE_NONE },
    { "gain",         &pathFollowConfig.lookahead_gain,    0.00f, 2.00f, 0.05f, 2, TUNE_RULE_NONE },
    { "min m",        &pathFollowConfig.lookahead_min_m,   0.05f, 1.00f, 0.01f, 2, TUNE_RULE_LOOKAHEAD_MIN },
    { "max m",        &pathFollowConfig.lookahead_max_m,   0.10f, 2.00f, 0.01f, 2, TUNE_RULE_LOOKAHEAD_MAX }
};

static const TuneItem_t tune_control[] = {
    { "pp gain",      &pathFollowConfig.pp_duty_per_curvature, 0.00f, 400.0f, 5.0f, 1, TUNE_RULE_NONE },
    { "lat kp",       &pathFollowConfig.lat_kp_duty_per_m,     0.00f, 600.0f, 5.0f, 1, TUNE_RULE_NONE },
    { "yaw kp",       &pathFollowConfig.yaw_kp_duty_per_rad,   0.00f, 400.0f, 5.0f, 1, TUNE_RULE_NONE },
    { "lat ki",       &pathFollowConfig.lat_ki_duty_per_m_s,   0.00f, 150.0f, 1.0f, 1, TUNE_RULE_NONE },
    { "lat kd",       &pathFollowConfig.lat_kd_duty_per_mps,   0.00f, 100.0f, 1.0f, 1, TUNE_RULE_NONE },
    { "lat i lim",    &pathFollowConfig.lat_integral_limit_m_s,0.00f, 2.00f, 0.05f, 2, TUNE_RULE_NONE },
    { "i min speed",  &pathFollowConfig.integral_min_speed_mps,0.00f, 0.30f, 0.01f, 2, TUNE_RULE_NONE },
    { "steer slew",   &pathFollowConfig.steer_slew_duty_per_s, 100.0f, 3000.0f, 50.0f, 0, TUNE_RULE_NONE }
};

static const TuneItem_t tune_speed[] = {
    { "v max",        &pathFollowConfig.v_max_mps,               0.05f, 1.00f, 0.01f, 2, TUNE_RULE_NONE },
    { "a lat max",    &pathFollowConfig.a_lat_max_mps2,          0.02f, 0.60f, 0.01f, 2, TUNE_RULE_NONE },
    { "end slow m",   &pathFollowConfig.end_slow_dist_m,         0.10f, 2.50f, 0.05f, 2, TUNE_RULE_NONE },
    { "speed kp",     &pathFollowConfig.speed_kp_duty_per_mps,   0.00f, 150.0f, 1.0f, 1, TUNE_RULE_NONE },
    { "speed ki",     &pathFollowConfig.speed_ki_duty_per_m,     0.00f, 80.0f,  1.0f, 1, TUNE_RULE_NONE },
    { "speed i lim",  &pathFollowConfig.speed_integral_limit_m,  0.00f, 2.00f, 0.05f, 2, TUNE_RULE_NONE },
    { "motor max",    &pathFollowConfig.motor_duty_max,          0.00f, 35.0f, 0.5f, 1, TUNE_RULE_NONE }
};

static const TuneCategoryInfo_t tune_categories[TUNE_CAT_COUNT] = {
    { "Safety",    tune_safety,    (uint8)ARRAY_SIZE(tune_safety)    },
    { "Lookahead", tune_lookahead, (uint8)ARRAY_SIZE(tune_lookahead) },
    { "Control",   tune_control,   (uint8)ARRAY_SIZE(tune_control)   },
    { "Speed",     tune_speed,     (uint8)ARRAY_SIZE(tune_speed)     }
};

static const char *system_items[] = {
    "WiFi reconnect",
    "IMU cal+zero",
    "Safety stop",
    "Reset tune",
    "Center steer"
};

static MenuPage_t menu_page = MENU_PAGE_MAIN;
static MenuEditMode_t menu_edit_mode = MENU_EDIT_NONE;
static uint8 menu_main_sel = 0;
static uint8 menu_param_cat = 0;
static uint8 menu_param_sel = 0;
static uint8 menu_param_detail = FALSE;
static uint8 menu_param_scroll[PARAM_CAT_COUNT] = { 0 };
static uint8 menu_tune_cat = 0;
static uint8 menu_tune_sel[TUNE_CAT_COUNT] = { 0 };
static uint8 menu_tune_detail = FALSE;
static uint8 menu_system_sel = 0;
static uint8 menu_need_full_redraw = TRUE;
static uint8 menu_need_redraw = TRUE;
static uint8 menu_need_value_redraw = FALSE;
static uint8 menu_need_footer_redraw = FALSE;
static uint8 menu_key12_ignore = FALSE;
static uint16 menu_refresh_tick = 0;
static uint16 menu_message_tick = 0;
static char menu_message[MENU_COLS + 1] = { 0 };
static float menu_edit_original = 0.0f;
static char menu_numeric_buf[MENU_NUMERIC_BUF_LEN + 1] = { 0 };
static uint8 menu_numeric_len = 0;
static uint8 menu_numeric_has_dot = FALSE;

static void menuSetMessage(const char *text);
static void menuRender(uint8 full_redraw);
static void menuRefreshLiveValues(void);
static void menuRefreshFooter(void);
static void menuHandleKeys(KeyEvent_t events[]);

static uint8 menuStrLen(const char *text)
{
    uint8 len = 0;

    if (text == NULL)
    {
        return 0;
    }

    while (text[len] != '\0' && len < 250u)
    {
        ++len;
    }
    return len;
}

static void menuCopyText(char *dst, uint8 dst_size, const char *src)
{
    uint8 i = 0;

    if (dst_size == 0)
    {
        return;
    }

    if (src != NULL)
    {
        while (src[i] != '\0' && i + 1u < dst_size)
        {
            dst[i] = src[i];
            ++i;
        }
    }
    dst[i] = '\0';
}

static void menuMakeBlank(char line[])
{
    uint8 i = 0;

    for (i = 0; i < MENU_COLS; ++i)
    {
        line[i] = ' ';
    }
    line[MENU_COLS] = '\0';
}

static void menuPutText(char line[], uint8 col, const char *text, uint8 max_len)
{
    uint8 i = 0;

    if (text == NULL || col >= MENU_COLS)
    {
        return;
    }

    while (text[i] != '\0' && i < max_len && (uint8)(col + i) < MENU_COLS)
    {
        line[col + i] = text[i];
        ++i;
    }
}

static void menuDrawRow(uint8 row, uint16 fg, uint16 bg, const char *text)
{
    char line[MENU_COLS + 1];

    if (row >= MENU_ROWS)
    {
        return;
    }

    menuMakeBlank(line);
    menuPutText(line, 0, text, MENU_COLS);
    ips200_set_color(fg, bg);
    ips200_show_string(0, (uint16)row * MENU_ROW_H, line);
}

static void menuClearBody(void)
{
    uint8 row = MENU_BODY_FIRST_ROW;

    for (row = MENU_BODY_FIRST_ROW; row <= MENU_BODY_LAST_ROW; ++row)
    {
        menuDrawRow(row, MENU_COLOR_FG, MENU_COLOR_BG, "");
    }
}

static void menuAppendChar(char *buf, uint8 size, char ch)
{
    uint8 len = menuStrLen(buf);

    if (len + 1u < size)
    {
        buf[len] = ch;
        buf[len + 1u] = '\0';
    }
}

static void menuAppendUInt(char *buf, uint8 size, uint32 value)
{
    char temp[11];
    uint8 len = 0;

    if (value == 0u)
    {
        menuAppendChar(buf, size, '0');
        return;
    }

    while (value > 0u && len < (uint8)sizeof(temp))
    {
        temp[len] = (char)('0' + (value % 10u));
        value /= 10u;
        ++len;
    }

    while (len > 0u)
    {
        --len;
        menuAppendChar(buf, size, temp[len]);
    }
}

static void menuFormatUInt(char *buf, uint8 size, uint32 value)
{
    if (size == 0u)
    {
        return;
    }
    buf[0] = '\0';
    menuAppendUInt(buf, size, value);
}

static void menuFormatInt(char *buf, uint8 size, int32 value)
{
    if (size == 0u)
    {
        return;
    }
    buf[0] = '\0';

    if (value < 0)
    {
        menuAppendChar(buf, size, '-');
        menuAppendUInt(buf, size, (uint32)(-value));
    }
    else
    {
        menuAppendUInt(buf, size, (uint32)value);
    }
}

static uint32 menuPow10(uint8 decimals)
{
    uint8 i = 0;
    uint32 scale = 1u;

    for (i = 0; i < decimals; ++i)
    {
        scale *= 10u;
    }
    return scale;
}

static void menuFormatFloat(char *buf, uint8 size, float value, uint8 decimals)
{
    float value_abs = value;
    uint32 scale = menuPow10(decimals);
    uint32 int_part = 0;
    uint32 frac_part = 0;
    uint32 div = 1u;
    uint8 i = 0;

    if (size == 0u)
    {
        return;
    }
    buf[0] = '\0';

    if (decimals > 6u)
    {
        decimals = 6u;
        scale = menuPow10(decimals);
    }

    if (value < 0.0f)
    {
        menuAppendChar(buf, size, '-');
        value_abs = -value;
    }

    int_part = (uint32)value_abs;
    frac_part = (uint32)((value_abs - (float)int_part) * (float)scale + 0.5f);
    if (frac_part >= scale)
    {
        ++int_part;
        frac_part -= scale;
    }

    menuAppendUInt(buf, size, int_part);

    if (decimals > 0u)
    {
        menuAppendChar(buf, size, '.');
        div = scale / 10u;
        for (i = 0; i < decimals; ++i)
        {
            menuAppendChar(buf, size, (char)('0' + (frac_part / div) % 10u));
            if (div > 1u)
            {
                div /= 10u;
            }
        }
    }
}

static const char *menuBoolText(uint8 value)
{
    return value ? "ON" : "OFF";
}

static const char *menuPathStatusText(uint8 status)
{
    switch (status)
    {
        case PATH_FOLLOW_IDLE:     return "IDLE";
        case PATH_FOLLOW_RUNNING:  return "RUN";
        case PATH_FOLLOW_FINISHED: return "DONE";
        case PATH_FOLLOW_ERROR:    return "ERROR";
        default:                   return "UNKNOWN";
    }
}

static float menuValueFloat(MenuValueId_t id)
{
    switch (id)
    {
        case VID_CAR_X:             return carPose.x;
        case VID_CAR_Y:             return carPose.y;
        case VID_CAR_YAW_DEG:       return carPose.yaw_deg;
        case VID_CAR_YAW_RAD:       return carPose.yaw;
        case VID_CAR_VX:            return carPose.vx;
        case VID_CAR_VY:            return carPose.vy;
        case VID_CAR_SPEED:         return carPose.speed;
        case VID_CAR_DISTANCE:      return carPose.distance;
        case VID_CAR_YAW_RATE:      return carPose.yaw_rate;
        case VID_CAR_STEER_ANGLE:   return carPose.steer_angle;
        case VID_CAR_GYRO_BIAS:     return carPose.gyro_bias;
        case VID_IMU_ROLL:          return imu_dat.roll;
        case VID_IMU_PITCH:         return imu_dat.pitch;
        case VID_IMU_YAW:           return imu_dat.yaw;
        case VID_IMU_GX:            return imu_dat.gx;
        case VID_IMU_GY:            return imu_dat.gy;
        case VID_IMU_GZ:            return imu_dat.gz;
        case VID_IMU_AX:            return imu_dat.ax;
        case VID_IMU_AY:            return imu_dat.ay;
        case VID_IMU_AZ:            return imu_dat.az;
        case VID_IMU_Q0:            return imu_dat.Q0;
        case VID_IMU_Q1:            return imu_dat.Q1;
        case VID_IMU_Q2:            return imu_dat.Q2;
        case VID_IMU_Q3:            return imu_dat.Q3;
        case VID_ENCODER_V:         return encoder_v;
        case VID_MOTOR_DUTY:        return motor_duty;
        case VID_STEER_DUTY:        return steer_duty;
        case VID_PATH_EY:           return pathFollower.e_y;
        case VID_PATH_EYAW:         return pathFollower.e_yaw;
        case VID_PATH_EY_D:         return pathFollower.e_y_derivative;
        case VID_PATH_LAT_I:        return pathFollower.lat_integral;
        case VID_PATH_LOOKAHEAD:    return pathFollower.lookahead;
        case VID_PATH_KAPPA:        return pathFollower.kappa_pp;
        case VID_PATH_STEER_OFFSET: return pathFollower.steer_offset;
        case VID_PATH_STEER_CMD:    return pathFollower.steer_duty_cmd;
        case VID_PATH_VREF:         return pathFollower.v_ref;
        case VID_PATH_SPEED_ERR:    return pathFollower.speed_error;
        case VID_PATH_SPEED_I:      return pathFollower.speed_integral;
        case VID_PATH_REMAIN:       return pathFollower.remaining_s;
        case VID_RECV_SPEED:        return recv_packet.speed;
        case VID_RECV_YAW:          return recv_packet.yaw;
        case VID_RECV_PITCH:        return recv_packet.pitch;
        case VID_RECV_ROLL:         return recv_packet.roll;
        case VID_RECV_DIST:         return recv_packet.distance;
        default:                    return 0.0f;
    }
}

static int32 menuValueInt(MenuValueId_t id)
{
    switch (id)
    {
        case VID_ENCODER_PULSE: return (int32)encoder_pulse;
        case VID_X6F_CH1:       return (int32)x6f_channel[0].out;
        case VID_X6F_CH2:       return (int32)x6f_channel[1].out;
        case VID_X6F_CH3:       return (int32)x6f_channel[2].out;
        case VID_X6F_CH4:       return (int32)x6f_channel[3].out;
        case VID_X6F_CH5:       return (int32)x6f_channel[4].out;
        case VID_X6F_CH6:       return (int32)x6f_channel[5].out;
        default:                return 0;
    }
}

static uint32 menuValueUInt(MenuValueId_t id)
{
    switch (id)
    {
        case VID_IMU_HOUR:       return (uint32)imu_dat.hh;
        case VID_IMU_MIN:        return (uint32)imu_dat.mn;
        case VID_IMU_SEC:        return (uint32)imu_dat.ss;
        case VID_IMU_MS:         return (uint32)imu_dat.ms;
        case VID_PATH_SELECTED:  return (uint32)pathFollower.selected_index;
        case VID_PATH_ACTIVE:    return (uint32)pathFollower.active_index;
        case VID_PATH_NEAREST:   return (uint32)pathFollower.nearest_index;
        case VID_PATH_TARGET:    return (uint32)pathFollower.target_index;
        case VID_RX_LEN:         return (uint32)g_uart_rx_frame.sta.len;
        case VID_RECV_ACTION:    return (uint32)recv_packet.action;
        default:                 return 0u;
    }
}

static uint8 menuValueBool(MenuValueId_t id)
{
    switch (id)
    {
        case VID_CAR_READY:   return carPose.position_ready;
        case VID_RC_ENABLE:   return rc_enable_flag;
        case VID_WIFI_POWER:  return atk_mw8266d_info.isPowerOn;
        case VID_WIFI_MODE:   return atk_mw8266d_info.isModeSet;
        case VID_WIFI_CONN:   return atk_mw8266d_info.isWiFiConnected;
        case VID_TCP_CONN:    return atk_mw8266d_info.isTcpConnected;
        case VID_SERIANET:    return atk_mw8266d_info.isSerianet;
        case VID_RX_FINISH:   return g_uart_rx_frame.sta.finsh;
        default:              return FALSE;
    }
}

static const char *menuValueText(MenuValueId_t id)
{
    switch (id)
    {
        case VID_PATH_STATUS: return menuPathStatusText(pathFollower.status);
        default:              return "";
    }
}

static void menuFormatParamValue(const ParamItem_t *item, char *buf, uint8 size)
{
    if (item == NULL)
    {
        menuCopyText(buf, size, "");
        return;
    }

    switch (item->type)
    {
        case MVAL_FLOAT:
            menuFormatFloat(buf, size, menuValueFloat(item->id), item->decimals);
            break;

        case MVAL_INT:
            menuFormatInt(buf, size, menuValueInt(item->id));
            break;

        case MVAL_UINT:
            menuFormatUInt(buf, size, menuValueUInt(item->id));
            break;

        case MVAL_BOOL:
            menuCopyText(buf, size, menuBoolText(menuValueBool(item->id)));
            break;

        case MVAL_TEXT:
            menuCopyText(buf, size, menuValueText(item->id));
            break;

        default:
            menuCopyText(buf, size, "");
            break;
    }
}

static uint8 menuDrawNameValue(uint8 row,
                               const char *name,
                               const char *value,
                               uint8 selected,
                               uint16 selected_bg)
{
    char line[MENU_COLS + 1];
    uint16 fg = selected ? MENU_COLOR_SELECT_FG : MENU_COLOR_FG;
    uint16 bg = selected ? selected_bg : MENU_COLOR_BG;
    uint8 name_len = menuStrLen(name);

    if (row > MENU_BODY_LAST_ROW)
    {
        return row;
    }

    menuMakeBlank(line);
    if (name_len > MENU_LABEL_COLS && row < MENU_BODY_LAST_ROW)
    {
        menuPutText(line, 0, name, MENU_LABEL_COLS);
        menuDrawRow(row, fg, bg, line);
        ++row;

        menuMakeBlank(line);
        menuPutText(line, 1, name + MENU_LABEL_COLS, (uint8)(MENU_LABEL_COLS - 1u));
        menuPutText(line, MENU_VALUE_COL, value, (uint8)(MENU_COLS - MENU_VALUE_COL));
        menuDrawRow(row, fg, bg, line);
        ++row;
    }
    else
    {
        menuPutText(line, 0, name, MENU_LABEL_COLS);
        menuPutText(line, MENU_VALUE_COL, value, (uint8)(MENU_COLS - MENU_VALUE_COL));
        menuDrawRow(row, fg, bg, line);
        ++row;
    }

    return row;
}

static uint8 menuValueRowForName(uint8 row, const char *name)
{
    uint8 name_len = menuStrLen(name);

    if (name_len > MENU_LABEL_COLS && row < MENU_BODY_LAST_ROW)
    {
        return (uint8)(row + 1u);
    }
    return row;
}

static uint8 menuNextRowForName(uint8 row, const char *name)
{
    uint8 name_len = menuStrLen(name);

    if (row > MENU_BODY_LAST_ROW)
    {
        return row;
    }

    if (name_len > MENU_LABEL_COLS && row < MENU_BODY_LAST_ROW)
    {
        return (uint8)(row + 2u);
    }
    return (uint8)(row + 1u);
}

static void menuDrawTextCell(uint8 row,
                             uint8 col,
                             uint8 cols,
                             uint16 fg,
                             uint16 bg,
                             const char *text)
{
    char line[MENU_COLS + 1];
    uint8 i = 0;

    if (row >= MENU_ROWS || col >= MENU_COLS || cols == 0u)
    {
        return;
    }

    if ((uint8)(col + cols) > MENU_COLS)
    {
        cols = (uint8)(MENU_COLS - col);
    }

    for (i = 0; i < cols; ++i)
    {
        line[i] = ' ';
    }
    line[cols] = '\0';
    menuPutText(line, 0, text, cols);

    ips200_set_color(fg, bg);
    ips200_show_string((uint16)col * 8u, (uint16)row * MENU_ROW_H, line);
}

static void menuDrawValueOnly(uint8 row,
                              const char *name,
                              const char *value,
                              uint8 selected,
                              uint16 selected_bg)
{
    uint8 value_row = menuValueRowForName(row, name);
    uint16 fg = selected ? MENU_COLOR_SELECT_FG : MENU_COLOR_FG;
    uint16 bg = selected ? selected_bg : MENU_COLOR_BG;

    if (value_row <= MENU_BODY_LAST_ROW)
    {
        menuDrawTextCell(value_row,
                         MENU_VALUE_COL,
                         (uint8)(MENU_COLS - MENU_VALUE_COL),
                         fg,
                         bg,
                         value);
    }
}

static void menuDrawHeader(const char *title)
{
    char line[MENU_COLS + 1];

    menuMakeBlank(line);
    menuPutText(line, 0, title, MENU_COLS);
    menuDrawRow(0, MENU_COLOR_HEADER, MENU_COLOR_BG, line);
}

static void menuDrawFooter(const char *hint)
{
    if (menu_message_tick > 0u)
    {
        menuDrawRow(MENU_FOOTER_ROW, MENU_COLOR_OK, MENU_COLOR_BG, menu_message);
    }
    else
    {
        menuDrawRow(MENU_FOOTER_ROW, MENU_COLOR_DIM, MENU_COLOR_BG, hint);
    }
}

static void menuSetMessage(const char *text)
{
    menuCopyText(menu_message, (uint8)sizeof(menu_message), text);
    menu_message_tick = MENU_MESSAGE_TICKS;
    menu_need_footer_redraw = TRUE;
}

static void menuNextParamCategory(void)
{
    ++menu_param_cat;
    if (menu_param_cat >= PARAM_CAT_COUNT)
    {
        menu_param_cat = 0;
    }
    menu_param_sel = menu_param_cat;
    menu_param_scroll[menu_param_cat] = 0;
    menu_need_full_redraw = TRUE;
}

static void menuPrevParamCategory(void)
{
    if (menu_param_cat == 0u)
    {
        menu_param_cat = PARAM_CAT_COUNT - 1u;
    }
    else
    {
        --menu_param_cat;
    }
    menu_param_sel = menu_param_cat;
    menu_param_scroll[menu_param_cat] = 0;
    menu_need_full_redraw = TRUE;
}

static void menuNextTuneCategory(void)
{
    ++menu_tune_cat;
    if (menu_tune_cat >= TUNE_CAT_COUNT)
    {
        menu_tune_cat = 0;
    }
    menu_need_full_redraw = TRUE;
}

static void menuPrevTuneCategory(void)
{
    if (menu_tune_cat == 0u)
    {
        menu_tune_cat = TUNE_CAT_COUNT - 1u;
    }
    else
    {
        --menu_tune_cat;
    }
    menu_need_full_redraw = TRUE;
}

static const TuneItem_t *menuCurrentTuneItem(void)
{
    const TuneCategoryInfo_t *cat = &tune_categories[menu_tune_cat];
    uint8 sel = menu_tune_sel[menu_tune_cat];

    if (sel >= cat->count)
    {
        return NULL;
    }
    return &cat->items[sel];
}

static float menuClipFloat(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static uint8 menuTuneValueAllowed(const TuneItem_t *item, float value)
{
    if (item == NULL)
    {
        return FALSE;
    }

    if (value < item->min_value || value > item->max_value)
    {
        return FALSE;
    }

    if (item->rule == TUNE_RULE_LOOKAHEAD_MIN && value > pathFollowConfig.lookahead_max_m)
    {
        return FALSE;
    }

    if (item->rule == TUNE_RULE_LOOKAHEAD_MAX && value < pathFollowConfig.lookahead_min_m)
    {
        return FALSE;
    }

    return TRUE;
}

static uint8 menuApplyTuneValue(const TuneItem_t *item, float value)
{
    if (!menuTuneValueAllowed(item, value))
    {
        menuSetMessage("Invalid range");
        buzzerBeep(BUZZER_LSS);
        return FALSE;
    }

    *(item->value) = value;
    return TRUE;
}

static void menuStartStepEdit(void)
{
    const TuneItem_t *item = menuCurrentTuneItem();

    if (item == NULL)
    {
        return;
    }

    menu_edit_original = *(item->value);
    menu_edit_mode = MENU_EDIT_STEP;
    menuSetMessage("L/R edit, OK save");
    menu_need_full_redraw = TRUE;
}

static void menuStartNumericEdit(void)
{
    const TuneItem_t *item = menuCurrentTuneItem();

    if (item == NULL)
    {
        return;
    }

    menu_edit_original = *(item->value);
    menu_numeric_len = 0;
    menu_numeric_has_dot = FALSE;
    menu_numeric_buf[0] = '\0';
    menu_edit_mode = MENU_EDIT_NUMERIC;
    menu_need_full_redraw = TRUE;
}

static void menuCancelEdit(void)
{
    const TuneItem_t *item = menuCurrentTuneItem();

    if (menu_edit_mode == MENU_EDIT_STEP && item != NULL)
    {
        *(item->value) = menu_edit_original;
    }

    menu_edit_mode = MENU_EDIT_NONE;
    menuSetMessage("Canceled");
    menu_need_full_redraw = TRUE;
}

static void menuConfirmStepEdit(void)
{
    menu_edit_mode = MENU_EDIT_NONE;
    menuSetMessage("Saved");
    buzzerBeep(BUZZER_S);
    menu_need_full_redraw = TRUE;
}

static void menuStepTuneValue(float delta)
{
    const TuneItem_t *item = menuCurrentTuneItem();
    float next_value = 0.0f;

    if (item == NULL)
    {
        return;
    }

    next_value = menuClipFloat(*(item->value) + delta, item->min_value, item->max_value);
    if (menuApplyTuneValue(item, next_value))
    {
        menu_need_value_redraw = TRUE;
    }
}

static void menuNumericAppend(char ch)
{
    if (menu_numeric_len >= MENU_NUMERIC_BUF_LEN)
    {
        return;
    }

    if (ch == '.')
    {
        if (menu_numeric_has_dot)
        {
            return;
        }
        menu_numeric_has_dot = TRUE;
    }

    menu_numeric_buf[menu_numeric_len] = ch;
    ++menu_numeric_len;
    menu_numeric_buf[menu_numeric_len] = '\0';
    menu_need_value_redraw = TRUE;
}

static uint8 menuNumericIsValid(void)
{
    if (menu_numeric_len == 0u)
    {
        return FALSE;
    }
    if (menu_numeric_len == 1u && menu_numeric_buf[0] == '.')
    {
        return FALSE;
    }
    return TRUE;
}

static void menuConfirmNumericEdit(void)
{
    const TuneItem_t *item = menuCurrentTuneItem();
    float value = 0.0f;

    if (item == NULL || !menuNumericIsValid())
    {
        menuSetMessage("Input empty");
        buzzerBeep(BUZZER_LSS);
        menu_key12_ignore = TRUE;
        return;
    }

    value = func_str_to_float(menu_numeric_buf);
    if (menuApplyTuneValue(item, value))
    {
        menu_edit_mode = MENU_EDIT_NONE;
        menu_key12_ignore = TRUE;
        menuSetMessage("Saved");
        buzzerBeep(BUZZER_S);
        menu_need_full_redraw = TRUE;
    }
    else
    {
        menu_key12_ignore = TRUE;
    }
}

static void menuWifiReconnect(void)
{
    uint8 wifi_ret = ATK_MW8266D_EOK;

    buzzerBeep(BUZZER_S);

    if (!atk_mw8266d_info.isModeSet)
    {
        wifi_ret = atk_mw8266d_set_mode(1);
        atk_mw8266d_info.isModeSet = (wifi_ret == ATK_MW8266D_EOK) ? 1 : 0;
    }

    if (atk_mw8266d_info.isModeSet && !atk_mw8266d_info.isWiFiConnected)
    {
        atk_mw8266d_sw_reset();
    }

    if (!atk_mw8266d_info.isWiFiConnected)
    {
        wifi_ret = atk_mw8266d_join_ap(WIFI_SSID, WIFI_PASSWORD);
        atk_mw8266d_info.isWiFiConnected = (wifi_ret == ATK_MW8266D_EOK) ? 1 : 0;
    }

    if (atk_mw8266d_info.isWiFiConnected && !atk_mw8266d_info.isTcpConnected)
    {
        atk_mw8266d_mux_config(0);
    }

    if (!atk_mw8266d_info.isTcpConnected)
    {
        wifi_ret = atk_mw8266d_connect_tcp_server(TCP_SERVER_IP, TCP_SERVER_PORT);
        atk_mw8266d_info.isTcpConnected = (wifi_ret == ATK_MW8266D_EOK) ? 1 : 0;
    }

    if (atk_mw8266d_ready())
    {
        wifi_ret = atk_mw8266d_enter_unvarnished();
        atk_mw8266d_info.isSerianet = (wifi_ret == ATK_MW8266D_EOK) ? 1 : 0;
    }

    if (atk_mw8266d_ready() && atk_mw8266d_info.isSerianet)
    {
        menuSetMessage("WiFi OK");
        buzzerBeep(BUZZER_FAST);
    }
    else
    {
        menuSetMessage("WiFi failed");
        buzzerBeep(BUZZER_LSS);
    }
}

static void menuSystemAction(void)
{
    switch (menu_system_sel)
    {
        case 0:
            menuSetMessage("WiFi busy...");
            menuRender(FALSE);
            menuWifiReconnect();
            break;

        case 1:
            menuSetMessage("IMU busy...");
            menuRender(FALSE);
            system_delay_ms(1000);
            buzzerBeep(BUZZER_SS);
            imuCalibrateAcc();
            imuZeroXY();
            imuZeroZ();
            buzzerBeep(BUZZER_L);
            menuSetMessage("IMU zero done");
            break;

        case 2:
            pathFollowerStop();
            motor_duty = 0.0f;
            steer_duty = (float)STEER_DUTY_M;
            menuSetMessage("Stopped");
            buzzerBeep(BUZZER_SS);
            break;

        case 3:
            pathFollowConfigResetDefaults();
            menuSetMessage("Tune reset");
            buzzerBeep(BUZZER_S);
            break;

        case 4:
            if (!pathFollowerIsRunning())
            {
                steer_duty = (float)STEER_DUTY_M;
                menuSetMessage("Steer center");
                buzzerBeep(BUZZER_S);
            }
            else
            {
                menuSetMessage("Stop path first");
                buzzerBeep(BUZZER_LSS);
            }
            break;

        default:
            break;
    }

    menu_need_full_redraw = TRUE;
}

static void menuEnterPreview(void)
{
    uint8 selected = pathFollower.selected_index;

    if (pathFollowerIsRunning())
    {
        menuSetMessage("Preview disabled RUN");
        buzzerBeep(BUZZER_LSS);
        return;
    }

    if (!trajectoryBuild(selected))
    {
        menuSetMessage("Build failed");
        buzzerBeep(BUZZER_LSS);
        return;
    }

    menu_page = MENU_PAGE_PREVIEW;
    menu_need_full_redraw = TRUE;
}

static void menuTrajectorySelectPrev(void)
{
    if (pathFollowerSelectPrev())
    {
        buzzerBeep(BUZZER_S);
    }
    else
    {
        menuSetMessage("Cannot select");
        buzzerBeep(BUZZER_LSS);
    }
    menu_need_redraw = TRUE;
}

static void menuTrajectorySelectNext(void)
{
    if (pathFollowerSelectNext())
    {
        buzzerBeep(BUZZER_S);
    }
    else
    {
        menuSetMessage("Cannot select");
        buzzerBeep(BUZZER_LSS);
    }
    menu_need_redraw = TRUE;
}

static void menuTrajectoryStartStop(void)
{
    if (pathFollowerIsRunning())
    {
        pathFollowerStop();
        menuSetMessage("Path stopped");
        buzzerBeep(BUZZER_SS);
    }
    else if (!rc_enable_flag && pathFollowerStart(&carPose))
    {
        menuSetMessage("Path started");
        buzzerBeep(BUZZER_S);
    }
    else
    {
        menuSetMessage("Start denied");
        buzzerBeep(BUZZER_LSS);
    }

    menu_need_redraw = TRUE;
}

static void menuHandleNumericKeys(KeyEvent_t events[])
{
    uint8 i = 0;

    for (i = 0; i < 9u; ++i)
    {
        if (events[i] == KEY_SHORT)
        {
            menuNumericAppend((char)('1' + i));
        }
    }

    if (events[10] == KEY_SHORT)
    {
        menuNumericAppend('0');
    }

    if (events[9] == KEY_SHORT)
    {
        menuNumericAppend('.');
    }

    if (events[11] == KEY_SHORT)
    {
        menuCancelEdit();
    }
    else if (events[11] == KEY_LONG)
    {
        menuConfirmNumericEdit();
    }
}

static void menuHandleStepKeys(KeyEvent_t events[])
{
    const TuneItem_t *item = menuCurrentTuneItem();

    if (item == NULL)
    {
        menu_edit_mode = MENU_EDIT_NONE;
        menu_need_full_redraw = TRUE;
        return;
    }

    if (events[3] == KEY_SHORT || events[3] == KEY_LONG)
    {
        menuStepTuneValue(-item->step);
    }

    if (events[5] == KEY_SHORT || events[5] == KEY_LONG)
    {
        menuStepTuneValue(item->step);
    }

    if (events[4] == KEY_SHORT)
    {
        menuConfirmStepEdit();
    }

    if (events[11] == KEY_SHORT)
    {
        menuCancelEdit();
    }
}

static void menuHandleMainKeys(KeyEvent_t events[])
{
    if (events[1] == KEY_SHORT || events[1] == KEY_LONG)
    {
        if (menu_main_sel == 0u) menu_main_sel = (uint8)(ARRAY_SIZE(main_items) - 1u);
        else --menu_main_sel;
        menu_need_redraw = TRUE;
    }

    if (events[7] == KEY_SHORT || events[7] == KEY_LONG)
    {
        ++menu_main_sel;
        if (menu_main_sel >= ARRAY_SIZE(main_items)) menu_main_sel = 0;
        menu_need_redraw = TRUE;
    }

    if (events[4] == KEY_SHORT)
    {
        if (menu_main_sel == 0u)
        {
            menu_page = MENU_PAGE_PARAMS;
            menu_param_detail = FALSE;
        }
        else if (menu_main_sel == 1u)
        {
            menu_page = MENU_PAGE_TUNE;
            menu_tune_detail = FALSE;
        }
        else if (menu_main_sel == 2u)
        {
            menu_page = MENU_PAGE_TRAJECTORY;
        }
        else
        {
            menu_page = MENU_PAGE_SYSTEM;
        }
        menu_need_full_redraw = TRUE;
    }
}

static void menuHandleParamsKeys(KeyEvent_t events[])
{
    const ParamCategoryInfo_t *cat = &param_categories[menu_param_cat];
    uint8 max_scroll = 0;

    if (!menu_param_detail)
    {
        if (events[1] == KEY_SHORT || events[1] == KEY_LONG)
        {
            if (menu_param_sel == 0u) menu_param_sel = PARAM_CAT_COUNT - 1u;
            else --menu_param_sel;
            menu_param_cat = menu_param_sel;
            menu_need_redraw = TRUE;
        }

        if (events[7] == KEY_SHORT || events[7] == KEY_LONG)
        {
            ++menu_param_sel;
            if (menu_param_sel >= PARAM_CAT_COUNT) menu_param_sel = 0;
            menu_param_cat = menu_param_sel;
            menu_need_redraw = TRUE;
        }

        if (events[3] == KEY_SHORT || events[3] == KEY_LONG)
        {
            menuPrevParamCategory();
        }

        if (events[5] == KEY_SHORT || events[5] == KEY_LONG)
        {
            menuNextParamCategory();
        }

        if (events[4] == KEY_SHORT)
        {
            menu_param_detail = TRUE;
            menu_param_scroll[menu_param_cat] = 0;
            menu_need_full_redraw = TRUE;
        }

        if (events[11] == KEY_SHORT)
        {
            menu_page = MENU_PAGE_MAIN;
            menu_need_full_redraw = TRUE;
        }
        return;
    }

    if (cat->count > (uint8)(MENU_BODY_LAST_ROW - MENU_BODY_FIRST_ROW + 1u))
    {
        max_scroll = (uint8)(cat->count - (MENU_BODY_LAST_ROW - MENU_BODY_FIRST_ROW + 1u));
    }

    if (events[1] == KEY_SHORT || events[1] == KEY_LONG)
    {
        if (menu_param_scroll[menu_param_cat] > 0u)
        {
            --menu_param_scroll[menu_param_cat];
            menu_need_redraw = TRUE;
        }
    }

    if (events[7] == KEY_SHORT || events[7] == KEY_LONG)
    {
        if (menu_param_scroll[menu_param_cat] < max_scroll)
        {
            ++menu_param_scroll[menu_param_cat];
            menu_need_redraw = TRUE;
        }
    }

    if (events[3] == KEY_SHORT || events[3] == KEY_LONG)
    {
        menuPrevParamCategory();
    }

    if (events[5] == KEY_SHORT || events[5] == KEY_LONG)
    {
        menuNextParamCategory();
    }

    if (events[11] == KEY_SHORT)
    {
        menu_param_detail = FALSE;
        menu_need_full_redraw = TRUE;
    }
}

static void menuHandleTuneKeys(KeyEvent_t events[])
{
    const TuneCategoryInfo_t *cat = &tune_categories[menu_tune_cat];

    if (!menu_tune_detail)
    {
        if (events[1] == KEY_SHORT || events[1] == KEY_LONG)
        {
            if (menu_tune_cat == 0u) menu_tune_cat = TUNE_CAT_COUNT - 1u;
            else --menu_tune_cat;
            menu_need_redraw = TRUE;
        }

        if (events[7] == KEY_SHORT || events[7] == KEY_LONG)
        {
            menuNextTuneCategory();
        }

        if (events[3] == KEY_SHORT || events[3] == KEY_LONG)
        {
            menuPrevTuneCategory();
        }

        if (events[5] == KEY_SHORT || events[5] == KEY_LONG)
        {
            menuNextTuneCategory();
        }

        if (events[4] == KEY_SHORT)
        {
            menu_tune_detail = TRUE;
            menu_need_full_redraw = TRUE;
        }

        if (events[11] == KEY_SHORT)
        {
            menu_page = MENU_PAGE_MAIN;
            menu_need_full_redraw = TRUE;
        }
        return;
    }

    if (events[1] == KEY_SHORT || events[1] == KEY_LONG)
    {
        if (menu_tune_sel[menu_tune_cat] == 0u) menu_tune_sel[menu_tune_cat] = (uint8)(cat->count - 1u);
        else --menu_tune_sel[menu_tune_cat];
        menu_need_redraw = TRUE;
    }

    if (events[7] == KEY_SHORT || events[7] == KEY_LONG)
    {
        ++menu_tune_sel[menu_tune_cat];
        if (menu_tune_sel[menu_tune_cat] >= cat->count) menu_tune_sel[menu_tune_cat] = 0;
        menu_need_redraw = TRUE;
    }

    if (events[3] == KEY_SHORT || events[3] == KEY_LONG)
    {
        menuPrevTuneCategory();
    }

    if (events[5] == KEY_SHORT || events[5] == KEY_LONG)
    {
        menuNextTuneCategory();
    }

    if (events[4] == KEY_SHORT)
    {
        menuStartStepEdit();
    }

    if (events[9] == KEY_SHORT)
    {
        menuStartNumericEdit();
    }

    if (events[11] == KEY_SHORT)
    {
        menu_tune_detail = FALSE;
        menu_need_full_redraw = TRUE;
    }
}

static void menuHandleTrajectoryKeys(KeyEvent_t events[])
{
    if (events[1] == KEY_SHORT || events[1] == KEY_LONG || events[3] == KEY_SHORT || events[3] == KEY_LONG)
    {
        menuTrajectorySelectPrev();
    }

    if (events[7] == KEY_SHORT || events[7] == KEY_LONG || events[5] == KEY_SHORT || events[5] == KEY_LONG)
    {
        menuTrajectorySelectNext();
    }

    if (events[4] == KEY_SHORT)
    {
        menuTrajectoryStartStop();
    }

    if (events[9] == KEY_SHORT)
    {
        menuEnterPreview();
    }

    if (events[11] == KEY_SHORT)
    {
        menu_page = MENU_PAGE_MAIN;
        menu_need_full_redraw = TRUE;
    }
}

static void menuHandleSystemKeys(KeyEvent_t events[])
{
    if (events[1] == KEY_SHORT || events[1] == KEY_LONG)
    {
        if (menu_system_sel == 0u) menu_system_sel = (uint8)(ARRAY_SIZE(system_items) - 1u);
        else --menu_system_sel;
        menu_need_redraw = TRUE;
    }

    if (events[7] == KEY_SHORT || events[7] == KEY_LONG)
    {
        ++menu_system_sel;
        if (menu_system_sel >= ARRAY_SIZE(system_items)) menu_system_sel = 0;
        menu_need_redraw = TRUE;
    }

    if (events[4] == KEY_SHORT)
    {
        menuSystemAction();
    }

    if (events[11] == KEY_SHORT)
    {
        menu_page = MENU_PAGE_MAIN;
        menu_need_full_redraw = TRUE;
    }
}

static void menuHandlePreviewKeys(KeyEvent_t events[])
{
    if (events[11] == KEY_SHORT)
    {
        menu_page = MENU_PAGE_TRAJECTORY;
        menu_need_full_redraw = TRUE;
    }
}

static void menuHandleKeys(KeyEvent_t events[])
{
    if (menu_edit_mode == MENU_EDIT_NUMERIC)
    {
        menuHandleNumericKeys(events);
        return;
    }

    if (menu_edit_mode == MENU_EDIT_STEP)
    {
        menuHandleStepKeys(events);
        return;
    }

    switch (menu_page)
    {
        case MENU_PAGE_MAIN:
            menuHandleMainKeys(events);
            break;

        case MENU_PAGE_PARAMS:
            menuHandleParamsKeys(events);
            break;

        case MENU_PAGE_TUNE:
            menuHandleTuneKeys(events);
            break;

        case MENU_PAGE_TRAJECTORY:
            menuHandleTrajectoryKeys(events);
            break;

        case MENU_PAGE_SYSTEM:
            menuHandleSystemKeys(events);
            break;

        case MENU_PAGE_PREVIEW:
            menuHandlePreviewKeys(events);
            break;

        default:
            break;
    }
}

static void menuRenderMain(void)
{
    uint8 i = 0;
    char text[MENU_COLS + 1];

    menuDrawHeader("Vehicle Menu");
    menuClearBody();

    for (i = 0; i < ARRAY_SIZE(main_items); ++i)
    {
        menuMakeBlank(text);
        menuPutText(text, 1, main_items[i], 24);
        menuDrawRow((uint8)(MENU_BODY_FIRST_ROW + i), menu_main_sel == i ? MENU_COLOR_SELECT_FG : MENU_COLOR_FG,
                    menu_main_sel == i ? MENU_COLOR_SELECT_BG : MENU_COLOR_BG, text);
    }

    menuDrawFooter("UP/DN SEL OK ENTER");
}

static void menuRenderParamsList(void)
{
    uint8 i = 0;
    char value[16];

    menuDrawHeader("Params Categories");
    menuClearBody();

    for (i = 0; i < PARAM_CAT_COUNT; ++i)
    {
        menuFormatUInt(value, (uint8)sizeof(value), (uint32)param_categories[i].count);
        menuDrawNameValue((uint8)(MENU_BODY_FIRST_ROW + i), param_categories[i].name, value,
                          menu_param_cat == i ? TRUE : FALSE, MENU_COLOR_SELECT_BG);
    }

    menuDrawFooter("OK VIEW  L/R CAT  BACK");
}

static void menuRenderParamsDetail(void)
{
    const ParamCategoryInfo_t *cat = &param_categories[menu_param_cat];
    uint8 row = MENU_BODY_FIRST_ROW;
    uint8 i = menu_param_scroll[menu_param_cat];
    char title[MENU_COLS + 1];
    char value[18];

    menuMakeBlank(title);
    menuPutText(title, 0, "Params ", 7);
    menuPutText(title, 7, cat->name, 18);
    menuDrawHeader(title);
    menuClearBody();

    while (i < cat->count && row <= MENU_BODY_LAST_ROW)
    {
        menuFormatParamValue(&cat->items[i], value, (uint8)sizeof(value));
        row = menuDrawNameValue(row, cat->items[i].name, value, FALSE, MENU_COLOR_SELECT_BG);
        ++i;
    }

    menuDrawFooter("L/R PAGE  UP/DN SCROLL");
}

static void menuRenderTuneList(void)
{
    uint8 i = 0;
    char value[16];

    menuDrawHeader("Tune Categories");
    menuClearBody();

    for (i = 0; i < TUNE_CAT_COUNT; ++i)
    {
        menuFormatUInt(value, (uint8)sizeof(value), (uint32)tune_categories[i].count);
        menuDrawNameValue((uint8)(MENU_BODY_FIRST_ROW + i), tune_categories[i].name, value,
                          menu_tune_cat == i ? TRUE : FALSE, MENU_COLOR_SELECT_BG);
    }

    menuDrawFooter("OK VIEW  L/R CAT  BACK");
}

static void menuRenderTuneDetail(void)
{
    const TuneCategoryInfo_t *cat = &tune_categories[menu_tune_cat];
    uint8 row = MENU_BODY_FIRST_ROW;
    uint8 i = 0;
    char title[MENU_COLS + 1];
    char value[18];
    uint16 select_bg = MENU_COLOR_SELECT_BG;

    menuMakeBlank(title);
    menuPutText(title, 0, "Tune ", 5);
    menuPutText(title, 5, cat->name, 20);
    menuDrawHeader(title);
    menuClearBody();

    if (menu_edit_mode == MENU_EDIT_STEP)
    {
        select_bg = MENU_COLOR_EDIT_BG;
    }

    for (i = 0; i < cat->count && row <= MENU_BODY_LAST_ROW; ++i)
    {
        menuFormatFloat(value, (uint8)sizeof(value), *(cat->items[i].value), cat->items[i].decimals);
        row = menuDrawNameValue(row, cat->items[i].name, value,
                                menu_tune_sel[menu_tune_cat] == i ? TRUE : FALSE,
                                select_bg);
    }

    if (menu_edit_mode == MENU_EDIT_STEP)
    {
        menuDrawFooter("L/R STEP OK SAVE BACK CAN");
    }
    else
    {
        menuDrawFooter("OK STEP  F NUM  BACK");
    }
}

static void menuRenderNumeric(void)
{
    const TuneItem_t *item = menuCurrentTuneItem();
    char value[18];

    menuDrawHeader("Direct Input");
    menuClearBody();

    if (item != NULL)
    {
        menuFormatFloat(value, (uint8)sizeof(value), *(item->value), item->decimals);
        menuDrawNameValue(MENU_BODY_FIRST_ROW, "item", item->name, FALSE, MENU_COLOR_SELECT_BG);
        menuDrawNameValue((uint8)(MENU_BODY_FIRST_ROW + 1u), "current", value, FALSE, MENU_COLOR_SELECT_BG);
        menuDrawNameValue((uint8)(MENU_BODY_FIRST_ROW + 3u), "input", menu_numeric_buf, TRUE, MENU_COLOR_EDIT_BG);
        menuFormatFloat(value, (uint8)sizeof(value), item->min_value, item->decimals);
        menuDrawNameValue((uint8)(MENU_BODY_FIRST_ROW + 5u), "min", value, FALSE, MENU_COLOR_SELECT_BG);
        menuFormatFloat(value, (uint8)sizeof(value), item->max_value, item->decimals);
        menuDrawNameValue((uint8)(MENU_BODY_FIRST_ROW + 6u), "max", value, FALSE, MENU_COLOR_SELECT_BG);
    }

    menuDrawFooter("K1-9 K11=0 K10=. L12=OK");
}

static void menuRenderTrajectory(void)
{
    uint8 count = trajectoryGetPresetCount();
    uint8 i = 0;
    uint8 row = MENU_BODY_FIRST_ROW;
    const TrajectoryPreset_t *preset;
    char value[18];
    char title[MENU_COLS + 1];

    menuDrawHeader("Trajectory");
    menuClearBody();

    for (i = 0; i < count && row <= (uint8)(MENU_BODY_FIRST_ROW + 6u); ++i)
    {
        preset = trajectoryGetPreset(i);
        if (preset != NULL)
        {
            menuFormatUInt(value, (uint8)sizeof(value), (uint32)i);
            row = menuDrawNameValue(row, preset->name, value,
                                    pathFollower.selected_index == i ? TRUE : FALSE,
                                    MENU_COLOR_SELECT_BG);
        }
    }

    row = 9u;
    row = menuDrawNameValue(row, "status", menuPathStatusText(pathFollower.status), FALSE, MENU_COLOR_SELECT_BG);
    menuFormatUInt(value, (uint8)sizeof(value), (uint32)pathFollower.selected_index);
    row = menuDrawNameValue(row, "selected", value, FALSE, MENU_COLOR_SELECT_BG);
    menuFormatUInt(value, (uint8)sizeof(value), (uint32)pathFollower.active_index);
    row = menuDrawNameValue(row, "active", value, FALSE, MENU_COLOR_SELECT_BG);
    menuFormatFloat(value, (uint8)sizeof(value), pathFollower.remaining_s, 3);
    row = menuDrawNameValue(row, "remain m", value, FALSE, MENU_COLOR_SELECT_BG);
    menuFormatFloat(value, (uint8)sizeof(value), pathFollower.e_y, 3);
    row = menuDrawNameValue(row, "e_y", value, FALSE, MENU_COLOR_SELECT_BG);
    menuFormatFloat(value, (uint8)sizeof(value), pathFollower.v_ref, 3);
    row = menuDrawNameValue(row, "v_ref", value, FALSE, MENU_COLOR_SELECT_BG);

    menuMakeBlank(title);
    if (pathFollowerIsRunning())
    {
        menuPutText(title, 0, "OK STOP  F PREVIEW OFF", 23);
    }
    else
    {
        menuPutText(title, 0, "OK START  F PREVIEW", 19);
    }
    menuDrawFooter(title);
}

static void menuRefreshParamsDetailValues(void)
{
    const ParamCategoryInfo_t *cat = &param_categories[menu_param_cat];
    uint8 row = MENU_BODY_FIRST_ROW;
    uint8 i = menu_param_scroll[menu_param_cat];
    char value[18];

    while (i < cat->count && row <= MENU_BODY_LAST_ROW)
    {
        menuFormatParamValue(&cat->items[i], value, (uint8)sizeof(value));
        menuDrawValueOnly(row, cat->items[i].name, value, FALSE, MENU_COLOR_SELECT_BG);
        row = menuNextRowForName(row, cat->items[i].name);
        ++i;
    }
}

static void menuRefreshTuneDetailValues(void)
{
    const TuneCategoryInfo_t *cat = &tune_categories[menu_tune_cat];
    uint8 row = MENU_BODY_FIRST_ROW;
    uint8 i = 0;
    char value[18];
    uint16 select_bg = (menu_edit_mode == MENU_EDIT_STEP) ? MENU_COLOR_EDIT_BG : MENU_COLOR_SELECT_BG;

    for (i = 0; i < cat->count && row <= MENU_BODY_LAST_ROW; ++i)
    {
        menuFormatFloat(value, (uint8)sizeof(value), *(cat->items[i].value), cat->items[i].decimals);
        menuDrawValueOnly(row,
                          cat->items[i].name,
                          value,
                          menu_tune_sel[menu_tune_cat] == i ? TRUE : FALSE,
                          select_bg);
        row = menuNextRowForName(row, cat->items[i].name);
    }
}

static void menuRefreshNumericValues(void)
{
    const TuneItem_t *item = menuCurrentTuneItem();
    char value[18];

    if (item == NULL)
    {
        return;
    }

    menuFormatFloat(value, (uint8)sizeof(value), *(item->value), item->decimals);
    menuDrawValueOnly((uint8)(MENU_BODY_FIRST_ROW + 1u), "current", value, FALSE, MENU_COLOR_SELECT_BG);
    menuDrawValueOnly((uint8)(MENU_BODY_FIRST_ROW + 3u), "input", menu_numeric_buf, TRUE, MENU_COLOR_EDIT_BG);
}

static void menuDrawTrajectoryFooter(void)
{
    char title[MENU_COLS + 1];

    menuMakeBlank(title);
    if (pathFollowerIsRunning())
    {
        menuPutText(title, 0, "OK STOP  F PREVIEW OFF", 23);
    }
    else
    {
        menuPutText(title, 0, "OK START  F PREVIEW", 19);
    }
    menuDrawFooter(title);
}

static void menuRefreshTrajectoryValues(void)
{
    char value[18];
    uint8 row = 9u;

    menuDrawValueOnly(row, "status", menuPathStatusText(pathFollower.status), FALSE, MENU_COLOR_SELECT_BG);
    row = menuNextRowForName(row, "status");

    menuFormatUInt(value, (uint8)sizeof(value), (uint32)pathFollower.selected_index);
    menuDrawValueOnly(row, "selected", value, FALSE, MENU_COLOR_SELECT_BG);
    row = menuNextRowForName(row, "selected");

    menuFormatUInt(value, (uint8)sizeof(value), (uint32)pathFollower.active_index);
    menuDrawValueOnly(row, "active", value, FALSE, MENU_COLOR_SELECT_BG);
    row = menuNextRowForName(row, "active");

    menuFormatFloat(value, (uint8)sizeof(value), pathFollower.remaining_s, 3);
    menuDrawValueOnly(row, "remain m", value, FALSE, MENU_COLOR_SELECT_BG);
    row = menuNextRowForName(row, "remain m");

    menuFormatFloat(value, (uint8)sizeof(value), pathFollower.e_y, 3);
    menuDrawValueOnly(row, "e_y", value, FALSE, MENU_COLOR_SELECT_BG);
    row = menuNextRowForName(row, "e_y");

    menuFormatFloat(value, (uint8)sizeof(value), pathFollower.v_ref, 3);
    menuDrawValueOnly(row, "v_ref", value, FALSE, MENU_COLOR_SELECT_BG);

    menuDrawTrajectoryFooter();
}

static void menuRefreshFooter(void)
{
    if (menu_edit_mode == MENU_EDIT_NUMERIC)
    {
        menuDrawFooter("K1-9 K11=0 K10=. L12=OK");
        return;
    }

    if (menu_edit_mode == MENU_EDIT_STEP)
    {
        menuDrawFooter("L/R STEP OK SAVE BACK CAN");
        return;
    }

    switch (menu_page)
    {
        case MENU_PAGE_MAIN:
            menuDrawFooter("UP/DN SEL OK ENTER");
            break;

        case MENU_PAGE_PARAMS:
            if (menu_param_detail)
            {
                menuDrawFooter("L/R PAGE  UP/DN SCROLL");
            }
            else
            {
                menuDrawFooter("OK VIEW  L/R CAT  BACK");
            }
            break;

        case MENU_PAGE_TUNE:
            if (menu_tune_detail)
            {
                menuDrawFooter("OK STEP  F NUM  BACK");
            }
            else
            {
                menuDrawFooter("OK VIEW  L/R CAT  BACK");
            }
            break;

        case MENU_PAGE_TRAJECTORY:
            menuDrawTrajectoryFooter();
            break;

        case MENU_PAGE_SYSTEM:
            menuDrawFooter("OK RUN  BACK");
            break;

        case MENU_PAGE_PREVIEW:
            menuDrawFooter("BACK EXIT");
            break;

        default:
            menuDrawFooter("");
            break;
    }
}

static void menuRenderSystem(void)
{
    uint8 i = 0;
    char text[MENU_COLS + 1];

    menuDrawHeader("System");
    menuClearBody();

    for (i = 0; i < ARRAY_SIZE(system_items); ++i)
    {
        menuMakeBlank(text);
        menuPutText(text, 1, system_items[i], 26);
        menuDrawRow((uint8)(MENU_BODY_FIRST_ROW + i),
                    menu_system_sel == i ? MENU_COLOR_SELECT_FG : MENU_COLOR_FG,
                    menu_system_sel == i ? MENU_COLOR_SELECT_BG : MENU_COLOR_BG,
                    text);
    }

    menuDrawFooter("OK RUN  BACK");
}

static int16 menuClampInt16(int16 value, int16 min_value, int16 max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static void menuDrawPreviewMarker(int16 x, int16 y, uint16 color)
{
    int16 x0 = menuClampInt16(x - 2, 0, (int16)(ips200_width_max - 1u));
    int16 x1 = menuClampInt16(x + 2, 0, (int16)(ips200_width_max - 1u));
    int16 y0 = menuClampInt16(y - 2, 0, (int16)(ips200_height_max - 1u));
    int16 y1 = menuClampInt16(y + 2, 0, (int16)(ips200_height_max - 1u));

    ips200_draw_line((uint16)x0, (uint16)y, (uint16)x1, (uint16)y, color);
    ips200_draw_line((uint16)x, (uint16)y0, (uint16)x, (uint16)y1, color);
}

static void menuRenderPreview(void)
{
    const TrajectoryPoint_t *points = trajectoryGetActivePoints();
    uint16 count = trajectoryGetActiveCount();
    uint16 i = 0;
    uint16 rect_x = 8u;
    uint16 rect_y = 32u;
    uint16 rect_w = 224u;
    uint16 rect_h = 256u;
    uint16 rect_x2 = (uint16)(rect_x + rect_w - 1u);
    uint16 rect_y2 = (uint16)(rect_y + rect_h - 1u);
    float min_x = 0.0f;
    float max_x = 0.0f;
    float min_y = 0.0f;
    float max_y = 0.0f;
    float span_x = 0.0f;
    float span_y = 0.0f;
    float scale_x = 1.0f;
    float scale_y = 1.0f;
    float scale = 1.0f;
    float mid_x = 0.0f;
    float mid_y = 0.0f;
    int16 last_px = 0;
    int16 last_py = 0;
    int16 px = 0;
    int16 py = 0;
    char title[MENU_COLS + 1];

    menuMakeBlank(title);
    menuPutText(title, 0, "Preview ", 8);
    menuPutText(title, 8, trajectoryGetActiveName(), 20);
    menuDrawHeader(title);
    menuClearBody();

    ips200_draw_line(rect_x, rect_y, rect_x2, rect_y, MENU_COLOR_DIM);
    ips200_draw_line(rect_x, rect_y2, rect_x2, rect_y2, MENU_COLOR_DIM);
    ips200_draw_line(rect_x, rect_y, rect_x, rect_y2, MENU_COLOR_DIM);
    ips200_draw_line(rect_x2, rect_y, rect_x2, rect_y2, MENU_COLOR_DIM);

    if (points == NULL || count < 2u)
    {
        menuDrawRow(10, MENU_COLOR_ERROR, MENU_COLOR_BG, "No trajectory");
        menuDrawFooter("BACK EXIT");
        return;
    }

    min_x = max_x = points[0].x;
    min_y = max_y = points[0].y;
    for (i = 1u; i < count; ++i)
    {
        if (points[i].x < min_x) min_x = points[i].x;
        if (points[i].x > max_x) max_x = points[i].x;
        if (points[i].y < min_y) min_y = points[i].y;
        if (points[i].y > max_y) max_y = points[i].y;
    }

    span_x = max_x - min_x;
    span_y = max_y - min_y;
    if (span_x < 0.10f) span_x = 0.10f;
    if (span_y < 0.10f) span_y = 0.10f;

    scale_x = ((float)rect_w - 24.0f) / span_y;
    scale_y = ((float)rect_h - 24.0f) / span_x;
    scale = (scale_x < scale_y) ? scale_x : scale_y;
    mid_x = (min_x + max_x) * 0.5f;
    mid_y = (min_y + max_y) * 0.5f;

    for (i = 0u; i < count; ++i)
    {
        px = (int16)((float)rect_x + (float)rect_w * 0.5f - (points[i].y - mid_y) * scale);
        py = (int16)((float)rect_y + (float)rect_h * 0.5f - (points[i].x - mid_x) * scale);
        px = menuClampInt16(px, (int16)(rect_x + 1u), (int16)(rect_x2 - 1u));
        py = menuClampInt16(py, (int16)(rect_y + 1u), (int16)(rect_y2 - 1u));

        if (i > 0u)
        {
            ips200_draw_line((uint16)last_px, (uint16)last_py, (uint16)px, (uint16)py, MENU_COLOR_PATH);
        }
        last_px = px;
        last_py = py;
    }

    px = (int16)((float)rect_x + (float)rect_w * 0.5f - (points[0].y - mid_y) * scale);
    py = (int16)((float)rect_y + (float)rect_h * 0.5f - (points[0].x - mid_x) * scale);
    px = menuClampInt16(px, (int16)(rect_x + 1u), (int16)(rect_x2 - 1u));
    py = menuClampInt16(py, (int16)(rect_y + 1u), (int16)(rect_y2 - 1u));
    menuDrawPreviewMarker(px, py, MENU_COLOR_OK);

    px = (int16)((float)rect_x + (float)rect_w * 0.5f - (points[count - 1u].y - mid_y) * scale);
    py = (int16)((float)rect_y + (float)rect_h * 0.5f - (points[count - 1u].x - mid_x) * scale);
    px = menuClampInt16(px, (int16)(rect_x + 1u), (int16)(rect_x2 - 1u));
    py = menuClampInt16(py, (int16)(rect_y + 1u), (int16)(rect_y2 - 1u));
    menuDrawPreviewMarker(px, py, MENU_COLOR_ERROR);

    ips200_draw_line(206u, 272u, 206u, 244u, MENU_COLOR_AXIS);
    ips200_draw_line(206u, 272u, 178u, 272u, MENU_COLOR_AXIS);
    menuDrawRow(18, MENU_COLOR_AXIS, MENU_COLOR_BG, "Axis: +X up  +Y left");
    menuDrawFooter("BACK EXIT");
}

static void menuRender(uint8 full_redraw)
{
    if (full_redraw)
    {
        ips200_set_color(MENU_COLOR_FG, MENU_COLOR_BG);
        ips200_clear();
    }

    if (menu_edit_mode == MENU_EDIT_NUMERIC)
    {
        menuRenderNumeric();
        return;
    }

    switch (menu_page)
    {
        case MENU_PAGE_MAIN:
            menuRenderMain();
            break;

        case MENU_PAGE_PARAMS:
            if (menu_param_detail) menuRenderParamsDetail();
            else menuRenderParamsList();
            break;

        case MENU_PAGE_TUNE:
            if (menu_tune_detail) menuRenderTuneDetail();
            else menuRenderTuneList();
            break;

        case MENU_PAGE_TRAJECTORY:
            menuRenderTrajectory();
            break;

        case MENU_PAGE_SYSTEM:
            menuRenderSystem();
            break;

        case MENU_PAGE_PREVIEW:
            menuRenderPreview();
            break;

        default:
            menuRenderMain();
            break;
    }
}

static void menuRefreshLiveValues(void)
{
    if (menu_edit_mode == MENU_EDIT_NUMERIC)
    {
        menuRefreshNumericValues();
        return;
    }

    if (menu_page == MENU_PAGE_PARAMS && menu_param_detail)
    {
        menuRefreshParamsDetailValues();
        return;
    }

    if (menu_page == MENU_PAGE_TUNE && menu_tune_detail)
    {
        menuRefreshTuneDetailValues();
        return;
    }

    if (menu_page == MENU_PAGE_TRAJECTORY)
    {
        menuRefreshTrajectoryValues();
        return;
    }
}

static uint8 menuNeedsLiveRefresh(void)
{
    if (menu_edit_mode != MENU_EDIT_NONE)
    {
        return TRUE;
    }

    if (menu_page == MENU_PAGE_PARAMS && menu_param_detail)
    {
        return TRUE;
    }

    if (menu_page == MENU_PAGE_TUNE && menu_tune_detail)
    {
        return TRUE;
    }

    if (menu_page == MENU_PAGE_TRAJECTORY)
    {
        return TRUE;
    }

    return FALSE;
}

void menuGuiInit(void)
{
    keyInit();
    ips200_set_font(IPS200_8X16_FONT);
    ips200_set_color(MENU_COLOR_FG, MENU_COLOR_BG);
    ips200_clear();

    menu_page = MENU_PAGE_MAIN;
    menu_edit_mode = MENU_EDIT_NONE;
    menu_need_full_redraw = TRUE;
    menu_need_redraw = TRUE;
}

void menuGuiTask(void)
{
    KeyEvent_t events[12];
    uint8 i = 0;
    uint8 event_found = FALSE;

    if (key_scan_flag < 2u)
    {
        return;
    }

    key_scan_flag = 0;

    for (i = 0; i < 12u; ++i)
    {
        events[i] = keyScan(&key[i]);
        if (events[i] != KEY_NONE)
        {
            event_found = TRUE;
        }
    }

    if (menu_key12_ignore)
    {
        if (key[11].state == 0u)
        {
            events[11] = KEY_NONE;
        }
        else
        {
            menu_key12_ignore = FALSE;
        }
    }

    if (menu_message_tick > 0u)
    {
        --menu_message_tick;
        if (menu_message_tick == 0u)
        {
            menu_need_footer_redraw = TRUE;
        }
    }

    if (event_found)
    {
        menuHandleKeys(events);
    }

    if (menu_need_full_redraw)
    {
        menuRender(TRUE);
        menu_need_full_redraw = FALSE;
        menu_need_redraw = FALSE;
        menu_need_value_redraw = FALSE;
        menu_need_footer_redraw = FALSE;
        menu_refresh_tick = 0;
        return;
    }

    if (menu_need_redraw)
    {
        menuRender(FALSE);
        menu_need_redraw = FALSE;
        menu_need_value_redraw = FALSE;
        menu_need_footer_redraw = FALSE;
        menu_refresh_tick = 0;
        return;
    }

    if (menu_need_footer_redraw)
    {
        menuRefreshFooter();
        menu_need_footer_redraw = FALSE;
    }

    if (menu_need_value_redraw)
    {
        menuRefreshLiveValues();
        menu_need_value_redraw = FALSE;
        menu_refresh_tick = 0;
        return;
    }

    ++menu_refresh_tick;
    if (menu_refresh_tick >= MENU_REFRESH_TICKS)
    {
        menu_refresh_tick = 0;
        if (menuNeedsLiveRefresh())
        {
            menuRefreshLiveValues();
        }
    }
}
