#ifndef CODE_TRAJECTORY_POINTS_H_
#define CODE_TRAJECTORY_POINTS_H_

/*
 * 车端预置控制点。
 *
 * 坐标系与 carPose 完全一致：
 *   - 单位：m
 *   - 原点：开机完成 IMU 置零时的小车位置
 *   - +X：开机时车头方向
 *   - +Y：开机时车体左侧
 *
 * 3m x 6m 场地内建议把 x 控制在 0~5m，把 y 控制在 -0.8~0.8m，
 * 给场地边缘、定位误差和舵机响应留出安全余量。
 *
 * 本头文件用于存放控制点数据，当前只由 trajectory.c 包含，避免 static 数据在多个编译单元中重复生成。
 */

#include "trajectory.h"

#define TRAJECTORY_LINE_POINT_COUNT     (5u)
#define TRAJECTORY_SOFT_S_POINT_COUNT   (6u)
#define TRAJECTORY_WIDE_S_POINT_COUNT   (6u)
#define TRAJECTORY_PRESET_COUNT         (3u)

static const TrajectoryControlPoint_t trajectory_line_points[TRAJECTORY_LINE_POINT_COUNT] = {
    { 0.00f,  0.00f },
    { 1.00f,  0.00f },
    { 2.00f,  0.00f },
    { 3.00f,  0.00f },
    { 4.00f,  0.00f }
};

static const TrajectoryControlPoint_t trajectory_soft_s_points[TRAJECTORY_SOFT_S_POINT_COUNT] = {
    { 0.00f,  0.00f },
    { 0.80f,  0.35f },
    { 1.60f, -0.35f },
    { 2.40f,  0.35f },
    { 3.20f, -0.30f },
    { 4.20f,  0.00f }
};

static const TrajectoryControlPoint_t trajectory_wide_s_points[TRAJECTORY_WIDE_S_POINT_COUNT] = {
    { 0.00f,  0.00f },
    { 0.80f,  0.60f },
    { 1.60f, -0.60f },
    { 2.40f,  0.60f },
    { 3.20f, -0.60f },
    { 4.40f,  0.00f }
};

static const TrajectoryPreset_t trajectory_presets[TRAJECTORY_PRESET_COUNT] = {
    { trajectory_line_points,   (uint8)TRAJECTORY_LINE_POINT_COUNT,   "Line"   },
    { trajectory_soft_s_points, (uint8)TRAJECTORY_SOFT_S_POINT_COUNT, "Soft S" },
    { trajectory_wide_s_points, (uint8)TRAJECTORY_WIDE_S_POINT_COUNT, "Wide S" }
};

#endif /* CODE_TRAJECTORY_POINTS_H_ */
