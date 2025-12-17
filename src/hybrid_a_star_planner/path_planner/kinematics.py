import math
import numpy as np
from typing import List, Tuple
from ..config import CarConfig, CostConfig, AlgorithmConfig
from ..core.node import HybridAStarNode
from ..core.util import normalize_angle, get_hybrid_grid_index


def generate_motion_commands() -> List[Tuple[float, int]]:
    """生成离散化的转向角和方向动作集：[(steer_angle, direction), ...]"""
    motion_commands = []
    direction = 1  # 前进

    # 转向角离散化
    steer_angles = np.linspace(
        -CarConfig.MAX_STEER_ANGLE,
        CarConfig.MAX_STEER_ANGLE,
        2 * CarConfig.STEER_PRECISION + 1,
    )

    for steer in steer_angles:
        # 前进 (Forward)
        motion_commands.append((steer, direction))
        # 后退 (Reverse)
        motion_commands.append((steer, -direction))

    return motion_commands


def simulate_trajectory(
    start_state: List[float],
    steer_angle: float,
    direction: int,
    sim_length: float,
    sim_step: float,
) -> List[List[float]]:
    """使用自行车模型模拟给定动作的轨迹 [(x, y, yaw), ...]"""
    traj = [start_state]  # [x, y, yaw]
    current_x, current_y, current_yaw = start_state

    num_steps = int(sim_length / sim_step)

    for _ in range(num_steps):
        # 运动学模型
        current_x += direction * sim_step * math.cos(current_yaw)
        current_y += direction * sim_step * math.sin(current_yaw)

        # yaw 变化
        delta_yaw = direction * sim_step / CarConfig.WHEEL_BASE * math.tan(steer_angle)
        current_yaw = normalize_angle(current_yaw + delta_yaw)

        traj.append([current_x, current_y, current_yaw])

    return traj[1:]  # 移除起始状态


def kinematic_simulation_node(
    current_node: HybridAStarNode, motion_command: Tuple[float, int], map_params
) -> HybridAStarNode | None:
    """模拟一个新节点并计算其成本和索引"""
    from .collision_check import is_valid

    steer_angle, direction = motion_command

    # 1. 模拟轨迹
    new_traj = simulate_trajectory(
        current_node.traj[-1],
        steer_angle,
        direction,
        AlgorithmConfig.SIMULATION_LENGTH,
        AlgorithmConfig.SIMULATION_STEP,
    )

    if not new_traj:
        return None  # 模拟失败
    segment_traj = [current_node.traj[-1]] + new_traj

    # 2. 检查有效性 (边界和碰撞)
    last_x, last_y, last_yaw = segment_traj[-1]
    grid_index = get_hybrid_grid_index(last_x, last_y, last_yaw)

    # 检查是否在地图边界内且无碰撞
    if not is_valid(segment_traj, grid_index, map_params):
        return None

    # 3. 计算成本
    cost = calculate_simulated_path_cost(
        current_node, steer_angle, direction, AlgorithmConfig.SIMULATION_LENGTH
    )

    # 4. 创建新节点
    return HybridAStarNode(
        grid_index=grid_index,
        traj=segment_traj,
        steering_angle=steer_angle,
        direction=direction,
        cost=current_node.cost + cost,  # 总成本
        parent_index=current_node.index,
    )


def calculate_simulated_path_cost(
    current_node: HybridAStarNode, steer_angle: float, direction: int, sim_length: float
) -> float:
    """计算一步运动学模拟的成本增量"""
    cost_increment = 0.0

    # 1. 距离成本
    if direction == 1:
        cost_increment += sim_length
    else:
        cost_increment += sim_length * CostConfig.REVERSE

    # 2. 换向惩罚
    if current_node.direction != direction:
        # 确保这不是起始节点（起始节点direction=1，parent_index=自身index）
        if current_node.parent_index != current_node.index:
            cost_increment += CostConfig.DIRECTION_CHANGE

    # 3. 转向角大小惩罚
    cost_increment += abs(steer_angle) * CostConfig.STEER_ANGLE

    # 4. 转向角变化惩罚
    cost_increment += (
        abs(steer_angle - current_node.steering_angle) * CostConfig.STEER_ANGLE_CHANGE
    )

    return cost_increment
