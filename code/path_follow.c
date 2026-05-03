#include "zf_common_headfile.h"
#include "define.h"
#include "motion.h"
#include "path_follow.h"

volatile PathFollowerState_t pathFollower = {
    .selected_index = 0,
    .active_index = 0,
    .status = PATH_FOLLOW_IDLE
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

static void resetControllerState(void)
{
    pathFollower.nearest_index = 0;
    pathFollower.target_index = 0;
    pathFollower.e_y = 0.0f;
    pathFollower.e_yaw = 0.0f;
    pathFollower.e_y_derivative = 0.0f;
    pathFollower.lat_integral = 0.0f;
    pathFollower.lookahead = PATH_LOOKAHEAD_MIN_M;
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
    if (start_dist > PATH_START_MAX_DIST_M)
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
    float v_ref = PATH_V_MAX_MPS;
    float max_kappa = maxAbsCurvatureAhead(points, count, nearest, target);

    if (max_kappa > 1e-4f)
    {
        float v_curve = sqrtf(PATH_A_LAT_MAX_MPS2 / max_kappa);
        if (v_curve < v_ref)
        {
            v_ref = v_curve;
        }
    }

    if (remaining_s < PATH_END_SLOW_DIST_M && PATH_END_SLOW_DIST_M > 1e-4f)
    {
        v_ref *= clipFloat(remaining_s / PATH_END_SLOW_DIST_M, 0.0f, 1.0f);
    }

    return clipFloat(v_ref, 0.0f, PATH_V_MAX_MPS);
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
    if (nearest_d2 > PATH_LOST_MAX_DIST_M * PATH_LOST_MAX_DIST_M)
    {
        errorFollower();
        return;
    }

    float path_length = trajectoryGetActiveLength();
    float remaining_s = path_length - points[nearest].s;
    if (remaining_s <= PATH_FINISH_DIST_M || nearest + 1 >= count)
    {
        finishFollower();
        return;
    }

    float lookahead = clipFloat(PATH_LOOKAHEAD_BASE_M + PATH_LOOKAHEAD_GAIN * fabsf(speed),
                                PATH_LOOKAHEAD_MIN_M,
                                PATH_LOOKAHEAD_MAX_M);
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
    if (fabsf(speed) > PATH_INTEGRAL_MIN_SPEED_MPS)
    {
        lat_integral_candidate += e_y * dt;
        lat_integral_candidate = clipFloat(lat_integral_candidate,
                                           -PATH_LAT_INTEGRAL_LIMIT_M_S,
                                            PATH_LAT_INTEGRAL_LIMIT_M_S);
    }

    float steer_offset = PATH_PP_DUTY_PER_CURVATURE * kappa_pp
                       - PATH_LAT_KP_DUTY_PER_M * e_y
                       - PATH_YAW_KP_DUTY_PER_RAD * e_yaw
                       - PATH_LAT_KI_DUTY_PER_M_S * lat_integral_candidate
                       - PATH_LAT_KD_DUTY_PER_MPS * e_y_derivative;

    float steer_desired = (float)STEER_DUTY_M + steer_offset;
    float max_steer_step = PATH_STEER_SLEW_DUTY_PER_S * dt;
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
    speed_integral = clipFloat(speed_integral, -PATH_SPEED_INTEGRAL_LIMIT_M, PATH_SPEED_INTEGRAL_LIMIT_M);

    float motor_cmd = PATH_SPEED_KP_DUTY_PER_MPS * speed_error
                    + PATH_SPEED_KI_DUTY_PER_M * speed_integral;
    motor_cmd = clipFloat(motor_cmd, 0.0f, PATH_MOTOR_DUTY_MAX);

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
