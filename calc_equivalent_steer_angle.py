#!/usr/bin/env python3
"""
计算 Ackermann 转向下的自行车模型等效前轮转角。

当前 ins 算法使用：

    yaw_rate = v / L * tan(delta)

因此等效转角 delta 应按 tan(delta) 平均，而不是直接平均角度：

    delta_eq = atan((tan(delta_left) + tan(delta_right)) / 2)

约定：
    - 左转为正角度
    - 右转为负角度
    - 如果输入的是角度绝对值，可使用 --direction left/right 自动添加符号
"""

from __future__ import annotations

import argparse
import math


def equivalent_angle(angle_left: float, angle_right: float, unit: str = "deg") -> float:
    """返回等效前轮转角，输出单位与输入 unit 一致。"""
    if unit == "deg":
        left_rad = math.radians(angle_left)
        right_rad = math.radians(angle_right)
    elif unit == "rad":
        left_rad = angle_left
        right_rad = angle_right
    else:
        raise ValueError(f"unsupported unit: {unit}")

    tan_eq = 0.5 * (math.tan(left_rad) + math.tan(right_rad))
    eq_rad = math.atan(tan_eq)

    return math.degrees(eq_rad) if unit == "deg" else eq_rad


def signed_by_direction(angle_left: float, angle_right: float, direction: str) -> tuple[float, float]:
    if direction == "left":
        return abs(angle_left), abs(angle_right)
    if direction == "right":
        return -abs(angle_left), -abs(angle_right)
    return angle_left, angle_right


def prompt_float(prompt: str) -> float:
    while True:
        text = input(prompt).strip()
        try:
            return float(text)
        except ValueError:
            print("输入无效，请输入数字。")


def interactive_args() -> argparse.Namespace:
    print("等效前轮转角计算")
    print("角度约定：左转为正，右转为负。若输入绝对值，请选择转向方向。")

    angle_left = prompt_float("左前轮与车身中轴夹角：")
    angle_right = prompt_float("右前轮与车身中轴夹角：")

    direction = input("转向方向 [left/right/none，默认 left]：").strip().lower() or "left"
    if direction not in ("left", "right", "none"):
        print("未知方向，按 none 处理。")
        direction = "none"

    return argparse.Namespace(
        angle_left=angle_left,
        angle_right=angle_right,
        unit="deg",
        direction=direction,
        wheelbase=None,
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="计算自行车模型等效前轮转角")
    parser.add_argument("angle_left", nargs="?", type=float, help="左前轮与车身中轴夹角")
    parser.add_argument("angle_right", nargs="?", type=float, help="右前轮与车身中轴夹角")
    parser.add_argument(
        "--unit",
        choices=("deg", "rad"),
        default="deg",
        help="输入和输出单位，默认 deg",
    )
    parser.add_argument(
        "--direction",
        choices=("left", "right", "none"),
        default="none",
        help="若输入为角度绝对值，用该参数指定转向方向；left 为正，right 为负",
    )
    parser.add_argument(
        "--wheelbase",
        type=float,
        help="可选：前后轴距，单位 m；提供后会同时输出估计转弯半径",
    )
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    if args.angle_left is None or args.angle_right is None:
        args = interactive_args()

    angle_left, angle_right = signed_by_direction(
        args.angle_left,
        args.angle_right,
        args.direction,
    )

    eq = equivalent_angle(angle_left, angle_right, args.unit)
    eq_rad = math.radians(eq) if args.unit == "deg" else eq
    eq_deg = math.degrees(eq) if args.unit == "rad" else eq

    print()
    print(f"左前轮角: {angle_left:.6f} {args.unit}")
    print(f"右前轮角: {angle_right:.6f} {args.unit}")
    print(f"等效前轮转角: {eq_deg:.6f} deg")
    print(f"等效前轮转角: {eq_rad:.6f} rad")

    if args.wheelbase is not None:
        tan_eq = math.tan(eq_rad)
        if abs(tan_eq) < 1e-9:
            print("估计转弯半径: inf")
        else:
            radius = args.wheelbase / tan_eq
            print(f"估计转弯半径: {radius:.6f} m")

    print()
    print("可填入 ins.h 的示例：")
    if eq_rad >= 0.0:
        print(f"#define INS_STEER_LEFT_MAX_RAD     ({eq_rad:.7f}f)")
    else:
        print(f"#define INS_STEER_RIGHT_MAX_RAD    ({eq_rad:.7f}f)")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
