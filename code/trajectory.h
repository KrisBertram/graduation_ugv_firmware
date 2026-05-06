#ifndef CODE_TRAJECTORY_H_
#define CODE_TRAJECTORY_H_

#include "zf_common_headfile.h"

/*
 * 轨迹生成参数。
 *
 * TRAJ_SAMPLE_STEP_M：
 *   轨迹重采样间距，单位 m。调小后轨迹点更密、曲率和航向变化更细腻，但占用更多 RAM
 *   并增加最近点搜索计算量；调大后点数减少，但弯道处可能变粗糙。
 *   3m x 6m 场地推荐 0.03~0.08m，默认 0.05m。
 */
#ifndef TRAJ_SAMPLE_STEP_M
#define TRAJ_SAMPLE_STEP_M      (0.05f)
#endif

/*
 * TRAJ_MAX_POINTS：
 *   活动轨迹最大离散点数。6m 场地按 0.05m 采样约 120 点，S 形会略多，256 留有余量。
 *   如果后续轨迹明显超过 6m，可按 “路径长度 / TRAJ_SAMPLE_STEP_M + 余量” 增大。
 */
#ifndef TRAJ_MAX_POINTS
#define TRAJ_MAX_POINTS         (256u)
#endif

/*
 * TRAJ_CATMULL_ALPHA：
 *   Catmull-Rom 参数化系数。0.5 为 centripetal Catmull-Rom，可减少控制点距离不均时的自交和过冲。
 *   一般保持 0.5，不建议上板调参时修改。
 */
#ifndef TRAJ_CATMULL_ALPHA
#define TRAJ_CATMULL_ALPHA      (0.5f)
#endif

typedef struct {
    float x;    // 控制点全局 X 坐标，单位 m
    float y;    // 控制点全局 Y 坐标，单位 m
} TrajectoryControlPoint_t;

typedef struct {
    float x;       // 轨迹点全局 X 坐标，单位 m
    float y;       // 轨迹点全局 Y 坐标，单位 m
    float yaw;     // 轨迹切线方向，单位 rad，逆时针为正
    float kappa;   // 曲率，单位 1/m，左转为正，右转为负
    float s;       // 从轨迹起点累计弧长，单位 m
    float v_ref;   // 此点建议目标速度，单位 m/s
} TrajectoryPoint_t;

typedef struct {
    const TrajectoryControlPoint_t *points;  // 控制点数组
    uint8 count;                             // 控制点数量
    const char *name;                        // 调试显示用轨迹名，建议仅使用 ASCII
    const char *desc;                        // 菜单信息栏显示用说明，建议仅使用 ASCII
} TrajectoryPreset_t;

uint8 trajectoryGetPresetCount(void);
const TrajectoryPreset_t *trajectoryGetPreset(uint8 index);

uint8 trajectoryBuild(uint8 preset_index);
const TrajectoryPoint_t *trajectoryGetActivePoints(void);
uint16 trajectoryGetActiveCount(void);
float trajectoryGetActiveLength(void);
const char *trajectoryGetActiveName(void);

#endif /* CODE_TRAJECTORY_H_ */
