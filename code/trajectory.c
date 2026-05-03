#include "zf_common_headfile.h"
#include "define.h"
#include "trajectory.h"
#include "trajectory_points.h"

typedef struct {
    float x;
    float y;
} TrajVec2_t;

static TrajectoryPoint_t active_points[TRAJ_MAX_POINTS];
static uint16 active_count = 0;
static uint8 active_preset_index = 0;

static float distance2d(TrajVec2_t a, TrajVec2_t b)
{
    float dx = b.x - a.x;
    float dy = b.y - a.y;
    return sqrtf(dx * dx + dy * dy);
}

static TrajVec2_t controlPointToVec(const TrajectoryControlPoint_t *pt)
{
    TrajVec2_t v = { pt->x, pt->y };
    return v;
}

static TrajVec2_t extrapolateBefore(TrajVec2_t p1, TrajVec2_t p2)
{
    TrajVec2_t v = { 2.0f * p1.x - p2.x, 2.0f * p1.y - p2.y };
    return v;
}

static TrajVec2_t extrapolateAfter(TrajVec2_t p1, TrajVec2_t p2)
{
    TrajVec2_t v = { 2.0f * p2.x - p1.x, 2.0f * p2.y - p1.y };
    return v;
}

static float catmullNextT(float ti, TrajVec2_t a, TrajVec2_t b)
{
    float chord = distance2d(a, b);
    if (chord < 1e-4f)
    {
        chord = 1e-4f;
    }
    return ti + powf(chord, TRAJ_CATMULL_ALPHA);
}

static TrajVec2_t lerpByT(TrajVec2_t a, TrajVec2_t b, float ta, float tb, float t)
{
    float denom = tb - ta;
    TrajVec2_t out;

    if (fabsf(denom) < 1e-6f)
    {
        out.x = a.x;
        out.y = a.y;
        return out;
    }

    float wa = (tb - t) / denom;
    float wb = (t - ta) / denom;
    out.x = wa * a.x + wb * b.x;
    out.y = wa * a.y + wb * b.y;
    return out;
}

static TrajVec2_t catmullRomPoint(TrajVec2_t p0, TrajVec2_t p1, TrajVec2_t p2, TrajVec2_t p3, float u)
{
    float t0 = 0.0f;
    float t1 = catmullNextT(t0, p0, p1);
    float t2 = catmullNextT(t1, p1, p2);
    float t3 = catmullNextT(t2, p2, p3);
    float t = t1 + (t2 - t1) * CLIP(u, 0.0f, 1.0f);

    TrajVec2_t a1 = lerpByT(p0, p1, t0, t1, t);
    TrajVec2_t a2 = lerpByT(p1, p2, t1, t2, t);
    TrajVec2_t a3 = lerpByT(p2, p3, t2, t3, t);
    TrajVec2_t b1 = lerpByT(a1, a2, t0, t2, t);
    TrajVec2_t b2 = lerpByT(a2, a3, t1, t3, t);
    return lerpByT(b1, b2, t1, t2, t);
}

static uint8 appendActivePoint(float x, float y, float s)
{
    if (active_count >= TRAJ_MAX_POINTS)
    {
        return FALSE;
    }

    active_points[active_count].x = x;
    active_points[active_count].y = y;
    active_points[active_count].yaw = 0.0f;
    active_points[active_count].kappa = 0.0f;
    active_points[active_count].s = s;
    active_points[active_count].v_ref = 0.0f;
    ++active_count;
    return TRUE;
}

static void updateYawAndCurvature(void)
{
    if (active_count < 2)
    {
        return;
    }

    for (uint16 i = 0; i < active_count; ++i)
    {
        uint16 prev = (i == 0) ? 0 : (uint16)(i - 1);
        uint16 next = (i + 1 >= active_count) ? (uint16)(active_count - 1) : (uint16)(i + 1);
        float dx = active_points[next].x - active_points[prev].x;
        float dy = active_points[next].y - active_points[prev].y;

        if (fabsf(dx) > 1e-5f || fabsf(dy) > 1e-5f)
        {
            active_points[i].yaw = atan2f(dy, dx);
        }
        else if (i > 0)
        {
            active_points[i].yaw = active_points[i - 1].yaw;
        }

        if (i > 0 && i + 1 < active_count)
        {
            TrajVec2_t a = { active_points[i - 1].x, active_points[i - 1].y };
            TrajVec2_t b = { active_points[i].x,     active_points[i].y     };
            TrajVec2_t c = { active_points[i + 1].x, active_points[i + 1].y };
            float ab = distance2d(a, b);
            float bc = distance2d(b, c);
            float ca = distance2d(c, a);
            float cross = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);

            if (ab > 1e-4f && bc > 1e-4f && ca > 1e-4f)
            {
                active_points[i].kappa = 2.0f * cross / (ab * bc * ca);
            }
        }
    }

    active_points[0].kappa = (active_count > 2) ? active_points[1].kappa : 0.0f;
    active_points[active_count - 1].kappa = (active_count > 2) ? active_points[active_count - 2].kappa : 0.0f;
}

uint8 trajectoryGetPresetCount(void)
{
    return (uint8)TRAJECTORY_PRESET_COUNT;
}

const TrajectoryPreset_t *trajectoryGetPreset(uint8 index)
{
    if (index >= trajectoryGetPresetCount())
    {
        return NULL;
    }
    return &trajectory_presets[index];
}

uint8 trajectoryBuild(uint8 preset_index)
{
    const TrajectoryPreset_t *preset = trajectoryGetPreset(preset_index);
    if (preset == NULL || preset->points == NULL || preset->count < 2)
    {
        active_count = 0;
        return FALSE;
    }

    active_count = 0;
    active_preset_index = preset_index;

    TrajVec2_t first = controlPointToVec(&preset->points[0]);
    if (!appendActivePoint(first.x, first.y, 0.0f))
    {
        active_count = 0;
        return FALSE;
    }

    float next_s = TRAJ_SAMPLE_STEP_M;
    float total_s = 0.0f;
    TrajVec2_t last_sample = first;

    for (uint8 seg = 0; seg + 1 < preset->count; ++seg)
    {
        TrajVec2_t p1 = controlPointToVec(&preset->points[seg]);
        TrajVec2_t p2 = controlPointToVec(&preset->points[seg + 1]);
        TrajVec2_t p0 = (seg == 0) ? extrapolateBefore(p1, p2) : controlPointToVec(&preset->points[seg - 1]);
        TrajVec2_t p3 = (seg + 2 >= preset->count) ? extrapolateAfter(p1, p2) : controlPointToVec(&preset->points[seg + 2]);

        float chord = distance2d(p1, p2);
        uint16 sample_count = (uint16)(chord / (TRAJ_SAMPLE_STEP_M * 0.25f)) + 8u;
        if (sample_count < 8u)
        {
            sample_count = 8u;
        }

        for (uint16 j = 1; j <= sample_count; ++j)
        {
            float u = (float)j / (float)sample_count;
            TrajVec2_t cur = catmullRomPoint(p0, p1, p2, p3, u);
            float seg_len = distance2d(last_sample, cur);

            if (seg_len > 1e-5f)
            {
                while (total_s + seg_len >= next_s)
                {
                    float ratio = (next_s - total_s) / seg_len;
                    TrajVec2_t out = {
                        last_sample.x + (cur.x - last_sample.x) * ratio,
                        last_sample.y + (cur.y - last_sample.y) * ratio
                    };

                    if (!appendActivePoint(out.x, out.y, next_s))
                    {
                        active_count = 0;
                        return FALSE;
                    }
                    next_s += TRAJ_SAMPLE_STEP_M;
                }
            }

            total_s += seg_len;
            last_sample = cur;
        }
    }

    TrajVec2_t final_pt = controlPointToVec(&preset->points[preset->count - 1]);
    if (active_count == 0)
    {
        active_count = 0;
        return FALSE;
    }

    float dx_last = final_pt.x - active_points[active_count - 1].x;
    float dy_last = final_pt.y - active_points[active_count - 1].y;
    if (sqrtf(dx_last * dx_last + dy_last * dy_last) > TRAJ_SAMPLE_STEP_M * 0.25f)
    {
        if (!appendActivePoint(final_pt.x, final_pt.y, total_s))
        {
            active_count = 0;
            return FALSE;
        }
    }
    else
    {
        active_points[active_count - 1].x = final_pt.x;
        active_points[active_count - 1].y = final_pt.y;
        active_points[active_count - 1].s = total_s;
    }

    if (active_count < 2)
    {
        active_count = 0;
        return FALSE;
    }

    updateYawAndCurvature();
    return TRUE;
}

const TrajectoryPoint_t *trajectoryGetActivePoints(void)
{
    return active_points;
}

uint16 trajectoryGetActiveCount(void)
{
    return active_count;
}

float trajectoryGetActiveLength(void)
{
    if (active_count == 0)
    {
        return 0.0f;
    }
    return active_points[active_count - 1].s;
}

const char *trajectoryGetActiveName(void)
{
    const TrajectoryPreset_t *preset = trajectoryGetPreset(active_preset_index);
    if (preset == NULL || preset->name == NULL)
    {
        return "None";
    }
    return preset->name;
}
