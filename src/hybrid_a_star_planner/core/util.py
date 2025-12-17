import math
from typing import Tuple, List
from .node import HybridAStarNode
from ..config import AlgorithmConfig


def normalize_angle(angle: float) -> float:
    """将角度归一化到 [-pi, pi] 范围内"""
    return math.atan2(math.sin(angle), math.cos(angle))


def get_hybrid_grid_index(x: float, y: float, yaw: float) -> Tuple[int, int, int]:
    """计算 Hybrid A* 节点的栅格索引 (x, y, yaw)"""
    x_idx = round(x / AlgorithmConfig.XY_RESOLUTION)
    y_idx = round(y / AlgorithmConfig.XY_RESOLUTION)
    yaw_idx = round(normalize_angle(yaw) / AlgorithmConfig.YAW_RESOLUTION)
    return (x_idx, y_idx, yaw_idx)


def get_holonomic_grid_index(
    x: float, y: float, xy_resolution: float
) -> Tuple[int, int]:
    """计算 Holonomic A* 节点的栅格索引 (x, y)"""
    x_idx = round(x / xy_resolution)
    y_idx = round(y / xy_resolution)
    return (x_idx, y_idx)


def get_hybrid_node_index(node: HybridAStarNode) -> Tuple[int, int, int]:
    """获取 Hybrid A* 节点的索引元组"""
    return node.grid_index


def get_path_cost_components(
    path_lengths: List[float], path_types: List[str]
) -> Tuple[float, float, float, float]:
    """
    计算 Reeds-Shepp 路径的成本组成部分
    返回 (距离成本, 换向成本, 转向角成本, 转向角变化成本)
    """
    from ..config import CostConfig, CarConfig

    distance_cost = 0.0
    for length in path_lengths:
        if length >= 0:
            distance_cost += abs(length)
        else:
            distance_cost += abs(length) * CostConfig.REVERSE

    # 方向变化成本
    direction_change_cost = 0.0
    for i in range(len(path_lengths) - 1):
        if path_lengths[i] * path_lengths[i + 1] < 0:
            direction_change_cost += CostConfig.DIRECTION_CHANGE

    # 转向角大小成本
    steer_angle_cost = 0.0
    for path_type in path_types:
        if path_type != "S":  # 非直线段
            steer_angle_cost += CarConfig.MAX_STEER_ANGLE * CostConfig.STEER_ANGLE

    # 转向角变化成本
    steer_angle_change_cost = 0.0
    turn_angles = []
    for path_type in path_types:
        if path_type == "R":
            turn_angles.append(-CarConfig.MAX_STEER_ANGLE)
        elif path_type == "L":
            turn_angles.append(CarConfig.MAX_STEER_ANGLE)
        else:
            turn_angles.append(0.0)

    for i in range(len(turn_angles) - 1):
        steer_angle_change_cost += (
            abs(turn_angles[i + 1] - turn_angles[i]) * CostConfig.STEER_ANGLE_CHANGE
        )

    return (
        distance_cost,
        direction_change_cost,
        steer_angle_cost,
        steer_angle_change_cost,
    )
