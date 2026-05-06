#include "zf_common_headfile.h"
#include "define.h"
#include "motion.h"
#include "path_follow.h"

volatile PathFollowerState_t pathFollower = {
    .selected_index = 0,
    .active_index = 0,
    .status = PATH_FOLLOW_IDLE
};

volatile PathFollowConfig_t pathFollowConfig = {
    .start_max_dist_m = PATH_START_MAX_DIST_M,
    .lost_max_dist_m = PATH_LOST_MAX_DIST_M,
    .finish_dist_m = PATH_FINISH_DIST_M,
    .lookahead_base_m = PATH_LOOKAHEAD_BASE_M,
    .lookahead_gain = PATH_LOOKAHEAD_GAIN,
    .lookahead_min_m = PATH_LOOKAHEAD_MIN_M,
    .lookahead_max_m = PATH_LOOKAHEAD_MAX_M,
    .pp_duty_per_curvature = PATH_PP_DUTY_PER_CURVATURE,
    .lat_kp_duty_per_m = PATH_LAT_KP_DUTY_PER_M,
    .yaw_kp_duty_per_rad = PATH_YAW_KP_DUTY_PER_RAD,
    .lat_ki_duty_per_m_s = PATH_LAT_KI_DUTY_PER_M_S,
    .lat_kd_duty_per_mps = PATH_LAT_KD_DUTY_PER_MPS,
    .lat_integral_limit_m_s = PATH_LAT_INTEGRAL_LIMIT_M_S,
    .integral_min_speed_mps = PATH_INTEGRAL_MIN_SPEED_MPS,
    .steer_slew_duty_per_s = PATH_STEER_SLEW_DUTY_PER_S,
    .v_max_mps = PATH_V_MAX_MPS,
    .a_lat_max_mps2 = PATH_A_LAT_MAX_MPS2,
    .end_slow_dist_m = PATH_END_SLOW_DIST_M,
    .speed_kp_duty_per_mps = PATH_SPEED_KP_DUTY_PER_MPS,
    .speed_ki_duty_per_m = PATH_SPEED_KI_DUTY_PER_M,
    .speed_integral_limit_m = PATH_SPEED_INTEGRAL_LIMIT_M,
    .motor_duty_max = PATH_MOTOR_DUTY_MAX
};

static float last_steer_duty = (float)STEER_DUTY_M;
static float last_e_y = 0.0f;
static uint8 last_e_y_valid = FALSE;

static float wrapAngle(float a)
{
    while (a > PI) a -= 2.0f * PI;
    while (a < -PI) a += 2.0f * PI;
    return a;
}

static float clipFloat(float x, float min_value, float max_value)
{
    if (x < min_value) return min_value;
    if (x > max_value) return max_value;
    return x;
}

void pathFollowConfigResetDefaults(void)
{
    pathFollowConfig.start_max_dist_m = PATH_START_MAX_DIST_M;
    pathFollowConfig.lost_max_dist_m = PATH_LOST_MAX_DIST_M;
    pathFollowConfig.finish_dist_m = PATH_FINISH_DIST_M;
    pathFollowConfig.lookahead_base_m = PATH_LOOKAHEAD_BASE_M;
    pathFollowConfig.lookahead_gain = PATH_LOOKAHEAD_GAIN;
    pathFollowConfig.lookahead_min_m = PATH_LOOKAHEAD_MIN_M;
    pathFollowConfig.lookahead_max_m = PATH_LOOKAHEAD_MAX_M;
    pathFollowConfig.pp_duty_per_curvature = PATH_PP_DUTY_PER_CURVATURE;
    pathFollowConfig.lat_kp_duty_per_m = PATH_LAT_KP_DUTY_PER_M;
    pathFollowConfig.yaw_kp_duty_per_rad = PATH_YAW_KP_DUTY_PER_RAD;
    pathFollowConfig.lat_ki_duty_per_m_s = PATH_LAT_KI_DUTY_PER_M_S;
    pathFollowConfig.lat_kd_duty_per_mps = PATH_LAT_KD_DUTY_PER_MPS;
    pathFollowConfig.lat_integral_limit_m_s = PATH_LAT_INTEGRAL_LIMIT_M_S;
    pathFollowConfig.integral_min_speed_mps = PATH_INTEGRAL_MIN_SPEED_MPS;
    pathFollowConfig.steer_slew_duty_per_s = PATH_STEER_SLEW_DUTY_PER_S;
    pathFollowConfig.v_max_mps = PATH_V_MAX_MPS;
    pathFollowConfig.a_lat_max_mps2 = PATH_A_LAT_MAX_MPS2;
    pathFollowConfig.end_slow_dist_m = PATH_END_SLOW_DIST_M;
    pathFollowConfig.speed_kp_duty_per_mps = PATH_SPEED_KP_DUTY_PER_MPS;
    pathFollowConfig.speed_ki_duty_per_m = PATH_SPEED_KI_DUTY_PER_M;
    pathFollowConfig.speed_integral_limit_m = PATH_SPEED_INTEGRAL_LIMIT_M;
    pathFollowConfig.motor_duty_max = PATH_MOTOR_DUTY_MAX;
}

static void resetControllerState(void)
{
    pathFollower.nearest_index = 0;
    pathFollower.target_index = 0;
    pathFollower.e_y = 0.0f;
    pathFollower.e_yaw = 0.0f;
    pathFollower.e_y_derivative = 0.0f;
    pathFollower.lat_integral = 0.0f;
    pathFollower.lookahead = pathFollowConfig.lookahead_min_m;
    pathFollower.kappa_pp = 0.0f;
    pathFollower.steer_offset = 0.0f;
    pathFollower.steer_duty_cmd = (float)STEER_DUTY_M;
    pathFollower.v_ref = 0.0f;
    pathFollower.speed_error = 0.0f;
    pathFollower.speed_integral = 0.0f;
    pathFollower.remaining_s = 0.0f;

    last_steer_duty = (float)STEER_DUTY_M;
    last_e_y = 0.0f;
    last_e_y_valid = FALSE;
}

static void stopOutputsToCenter(void)
{
    motor_duty = 0.0f;
    steer_duty = (float)STEER_DUTY_M;
    last_steer_duty = (float)STEER_DUTY_M;
}

static void finishFollower(void)
{
    stopOutputsToCenter();
    resetControllerState();
    pathFollower.status = PATH_FOLLOW_FINISHED;
}

static void errorFollower(void)
{
    stopOutputsToCenter();
    resetControllerState();
    pathFollower.status = PATH_FOLLOW_ERROR;
}

uint8 pathFollowerSelectPrev(void)
{
    uint8 count = trajectoryGetPresetCount();

    if (count == 0 || pathFollower.status == PATH_FOLLOW_RUNNING)
    {
        return FALSE;
    }

    if (pathFollower.selected_index == 0)
    {
        pathFollower.selected_index = (uint8)(count - 1);
    }
    else
    {
        --pathFollower.selected_index;
    }
    pathFollower.status = PATH_FOLLOW_IDLE;
    return TRUE;
}

uint8 pathFollowerSelectNext(void)
{
    uint8 count = trajectoryGetPresetCount();

    if (count == 0 || pathFollower.status == PATH_FOLLOW_RUNNING)
    {
        return FALSE;
    }

    ++pathFollower.selected_index;
    if (pathFollower.selected_index >= count)
    {
        pathFollower.selected_index = 0;
    }
    pathFollower.status = PATH_FOLLOW_IDLE;
    return TRUE;
}

uint8 pathFollowerStart(const volatile CarPoseState *pose)
{
    uint8 selected = pathFollower.selected_index;

    if (pathFollower.status == PATH_FOLLOW_RUNNING)
    {
        return FALSE;
    }

    if (!trajectoryBuild(selected))
    {
        errorFollower();
        return FALSE;
    }

    const TrajectoryPoint_t *points = trajectoryGetActivePoints();
    uint16 count = trajectoryGetActiveCount();
    if (points == NULL || count < 2)
    {
        errorFollower();
        return FALSE;
    }

    float dx = pose->x - points[0].x;
    float dy = pose->y - points[0].y;
    float start_dist = sqrtf(dx * dx + dy * dy);
    if (start_dist > pathFollowConfig.start_max_dist_m)
    {
        errorFollower();
        return FALSE;
    }

    resetControllerState();
    pathFollower.active_index = selected;
    pathFollower.status = PATH_FOLLOW_RUNNING;
    stopOutputsToCenter();
    return TRUE;
}

void pathFollowerStop(void)
{
    stopOutputsToCenter();
    resetControllerState();
    pathFollower.status = PATH_FOLLOW_IDLE;
}

uint8 pathFollowerIsRunning(void)
{
    return (pathFollower.status == PATH_FOLLOW_RUNNING) ? TRUE : FALSE;
}

static uint16 findNearestIndex(const TrajectoryPoint_t *points, uint16 count, float x, float y, float *best_d2)
{
    uint16 best = 0;
    float best_value = 1e30f;

    for (uint16 i = 0; i < count; ++i)
    {
        float dx = x - points[i].x;
        float dy = y - points[i].y;
        float d2 = dx * dx + dy * dy;

        if (d2 < best_value)
        {
            best_value = d2;
            best = i;
        }
    }

    if (best_d2 != NULL)
    {
        *best_d2 = best_value;
    }
    return best;
}

static uint16 findTargetIndex(const TrajectoryPoint_t *points, uint16 count, uint16 nearest, float lookahead)
{
    uint16 target = nearest;
    float start_s = points[nearest].s;

    while (target + 1 < count && points[target].s - start_s < lookahead)
    {
        ++target;
    }
    return target;
}

static float maxAbsCurvatureAhead(const TrajectoryPoint_t *points, uint16 count, uint16 from, uint16 to)
{
    float max_kappa = 0.0f;

    if (to >= count)
    {
        to = (uint16)(count - 1);
    }

    for (uint16 i = from; i <= to; ++i)
    {
        float k = fabsf(points[i].kappa);
        if (k > max_kappa)
        {
            max_kappa = k;
        }
    }
    return max_kappa;
}

static float calculateReferenceSpeed(const TrajectoryPoint_t *points,
                                     uint16 count,
                                     uint16 nearest,
                                     uint16 target,
                                     float remaining_s)
{
    float v_ref = pathFollowConfig.v_max_mps;
    float max_kappa = maxAbsCurvatureAhead(points, count, nearest, target);

    if (max_kappa > 1e-4f)
    {
        float v_curve = sqrtf(pathFollowConfig.a_lat_max_mps2 / max_kappa);
        if (v_curve < v_ref)
        {
            v_ref = v_curve;
        }
    }

    if (remaining_s < pathFollowConfig.end_slow_dist_m && pathFollowConfig.end_slow_dist_m > 1e-4f)
    {
        v_ref *= clipFloat(remaining_s / pathFollowConfig.end_slow_dist_m, 0.0f, 1.0f);
    }

    return clipFloat(v_ref, 0.0f, pathFollowConfig.v_max_mps);
}

void pathFollowerUpdate(const volatile CarPoseState *pose, float dt, uint8 rc_enable)
{
    if (rc_enable)
    {
        if (pathFollower.status == PATH_FOLLOW_RUNNING)
        {
            pathFollowerStop();
        }
        return;
    }

    if (pathFollower.status != PATH_FOLLOW_RUNNING)
    {
        return;
    }

    if (dt <= 0.0f)
    {
        return;
    }
    if (dt > INS_DT_MAX)
    {
        dt = INS_DT_MAX;
    }

    const TrajectoryPoint_t *points = trajectoryGetActivePoints();
    uint16 count = trajectoryGetActiveCount();
    if (points == NULL || count < 2)
    {
        errorFollower();
        return;
    }

    float x = pose->x;
    float y = pose->y;
    float yaw = pose->yaw;
    float speed = pose->speed;

    float nearest_d2 = 0.0f;
    uint16 nearest = findNearestIndex(points, count, x, y, &nearest_d2);
    if (nearest_d2 > pathFollowConfig.lost_max_dist_m * pathFollowConfig.lost_max_dist_m)
    {
        errorFollower();
        return;
    }

    float path_length = trajectoryGetActiveLength();
    float remaining_s = path_length - points[nearest].s;
    if (remaining_s <= pathFollowConfig.finish_dist_m || nearest + 1 >= count)
    {
        finishFollower();
        return;
    }

    float lookahead_min = pathFollowConfig.lookahead_min_m;
    float lookahead_max = pathFollowConfig.lookahead_max_m;
    if (lookahead_min > lookahead_max)
    {
        lookahead_max = lookahead_min;
    }
    float lookahead = clipFloat(pathFollowConfig.lookahead_base_m + pathFollowConfig.lookahead_gain * fabsf(speed),
                                lookahead_min,
                                lookahead_max);
    uint16 target = findTargetIndex(points, count, nearest, lookahead);

    float dx_target = points[target].x - x;
    float dy_target = points[target].y - y;
    float alpha = wrapAngle(atan2f(dy_target, dx_target) - yaw);
    float kappa_pp = 0.0f;
    if (lookahead > 1e-4f)
    {
        kappa_pp = 2.0f * sinf(alpha) / lookahead;
    }

    float dx_nearest = x - points[nearest].x;
    float dy_nearest = y - points[nearest].y;
    float e_y = -sinf(points[nearest].yaw) * dx_nearest + cosf(points[nearest].yaw) * dy_nearest;
    float e_yaw = wrapAngle(yaw - points[nearest].yaw);
    float e_y_derivative = 0.0f;

    if (last_e_y_valid && dt > 1e-4f)
    {
        e_y_derivative = (e_y - last_e_y) / dt;
    }
    last_e_y = e_y;
    last_e_y_valid = TRUE;

    float lat_integral_candidate = pathFollower.lat_integral;
    if (fabsf(speed) > pathFollowConfig.integral_min_speed_mps)
    {
        lat_integral_candidate += e_y * dt;
        lat_integral_candidate = clipFloat(lat_integral_candidate,
                                           -pathFollowConfig.lat_integral_limit_m_s,
                                            pathFollowConfig.lat_integral_limit_m_s);
    }

    float steer_offset = pathFollowConfig.pp_duty_per_curvature * kappa_pp
                       - pathFollowConfig.lat_kp_duty_per_m * e_y
                       - pathFollowConfig.yaw_kp_duty_per_rad * e_yaw
                       - pathFollowConfig.lat_ki_duty_per_m_s * lat_integral_candidate
                       - pathFollowConfig.lat_kd_duty_per_mps * e_y_derivative;

    float steer_desired = (float)STEER_DUTY_M + steer_offset;
    float max_steer_step = pathFollowConfig.steer_slew_duty_per_s * dt;
    float steer_step = clipFloat(steer_desired - last_steer_duty, -max_steer_step, max_steer_step);
    float steer_slew = last_steer_duty + steer_step;
    uint8 steer_saturated = (steer_slew < (float)STEER_DUTY_MIN || steer_slew > (float)STEER_DUTY_MAX) ? TRUE : FALSE;
    float steer_cmd = clipFloat(steer_slew, (float)STEER_DUTY_MIN, (float)STEER_DUTY_MAX);

    if (!steer_saturated)
    {
        pathFollower.lat_integral = lat_integral_candidate;
    }

    float v_ref = calculateReferenceSpeed(points, count, nearest, target, remaining_s);
    float speed_error = v_ref - speed;
    float speed_integral = pathFollower.speed_integral + speed_error * dt;
    speed_integral = clipFloat(speed_integral, -pathFollowConfig.speed_integral_limit_m, pathFollowConfig.speed_integral_limit_m);

    float motor_cmd = pathFollowConfig.speed_kp_duty_per_mps * speed_error
                    + pathFollowConfig.speed_ki_duty_per_m * speed_integral;
    motor_cmd = clipFloat(motor_cmd, 0.0f, pathFollowConfig.motor_duty_max);

    pathFollower.nearest_index = nearest;
    pathFollower.target_index = target;
    pathFollower.e_y = e_y;
    pathFollower.e_yaw = e_yaw;
    pathFollower.e_y_derivative = e_y_derivative;
    pathFollower.lookahead = lookahead;
    pathFollower.kappa_pp = kappa_pp;
    pathFollower.steer_offset = steer_offset;
    pathFollower.steer_duty_cmd = steer_cmd;
    pathFollower.v_ref = v_ref;
    pathFollower.speed_error = speed_error;
    pathFollower.speed_integral = speed_integral;
    pathFollower.remaining_s = remaining_s;

    if (pathFollower.status != PATH_FOLLOW_RUNNING)
    {
        return;
    }

    steer_duty = steer_cmd;
    motor_duty = motor_cmd;
    last_steer_duty = steer_cmd;
}
