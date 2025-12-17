import math
from typing import Optional
from heapdict import heapdict
from ..config import CarConfig
from ..core.node import HybridAStarNode
from ..core.map_data import MapParameters
from ..core.util import get_path_cost_components
import curves_generator.reeds_shepp as rsCurve


def calculate_reeds_shepp_cost(current_node: HybridAStarNode, rs_path) -> float:
    """计算 Reeds-Shepp 路径的完整成本 (G-Cost 增量 + 父节点成本)"""

    # 获取路径的四个成本组成部分
    distance_cost, direction_change_cost, steer_angle_cost, steer_angle_change_cost = (
        get_path_cost_components(rs_path.lengths, rs_path.ctypes)
    )

    # 总成本增量
    cost_increment = (
        distance_cost
        + direction_change_cost
        + steer_angle_cost
        + steer_angle_change_cost
    )

    return current_node.cost + cost_increment


def reeds_shepp_expansion(
    current_node: HybridAStarNode, goal_node: HybridAStarNode, map_params: MapParameters
) -> Optional[HybridAStarNode]:
    """
    尝试从当前节点到目标节点找到一条 Reeds-Shepp 路径作为启发式或最终路径。
    返回找到的路径节点，如果没有找到或有碰撞则返回 None。
    """
    from .collision_check import check_collision

    # 1. 提取起点和终点状态
    start_x, start_y, start_yaw = current_node.x, current_node.y, current_node.yaw
    goal_x, goal_y, goal_yaw = goal_node.x, goal_node.y, goal_node.yaw

    # 2. 计算最大曲率 (kappa = tan(max_steer) / L)
    max_curvature = math.tan(CarConfig.MAX_STEER_ANGLE) / CarConfig.WHEEL_BASE

    # 3. 查找所有可能的 Reeds-Shepp 路径
    rs_paths = rsCurve.calc_all_paths(
        start_x,
        start_y,
        start_yaw,
        goal_x,
        goal_y,
        goal_yaw,
        max_curvature,
        step_size=0.1,
    )

    if not rs_paths:
        return None

    # 4. 按成本排序所有路径
    cost_queue = heapdict()
    for path in rs_paths:
        # 仅使用增量成本进行排序
        cost_queue[path] = calculate_reeds_shepp_cost(current_node, path)

    # 5. 检查成本最低且无碰撞的路径
    while cost_queue:
        rs_path = cost_queue.popitem()[0]

        # 将 Reeds-Shepp 路径转换为轨迹格式
        traj = [
            [rs_path.x[k], rs_path.y[k], rs_path.yaw[k]] for k in range(len(rs_path.x))
        ]

        if not check_collision(traj, map_params):
            # 找到一条有效的路径
            final_cost = calculate_reeds_shepp_cost(current_node, rs_path)

            return HybridAStarNode(
                grid_index=goal_node.grid_index,
                traj=traj,
                steering_angle=0.0,  # Reeds-Shepp 路径本身不代表单个动作
                direction=1,  # 不重要，因为它是一个最终路径
                cost=final_cost,
                parent_index=current_node.index,
            )

    return None
