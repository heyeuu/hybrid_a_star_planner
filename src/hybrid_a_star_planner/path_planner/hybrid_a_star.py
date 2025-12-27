from typing import Tuple, List, Optional, Dict
from heapdict import heapdict
import matplotlib.pyplot as plt

from ..config import CostConfig
from ..core.node import HybridAStarNode
from ..core.map_data import MapParameters
from ..core.util import get_hybrid_grid_index
from .kinematics import generate_motion_commands, kinematic_simulation_node
from .reeds_shepp_path import reeds_shepp_expansion
from .holonomic_astar import calculate_holonomic_cost_with_obstacles


def run_hybrid_a_star(
    start_state: List[float],
    goal_state: List[float],
    map_params: MapParameters,
    plot_enabled: bool = False,
) -> Optional[Tuple[List[float], List[float], List[float], Dict]]:
    """
    执行 Hybrid A* 路径规划算法
    :param start_state: 起点 [x, y, yaw]
    :param goal_state: 终点 [x, y, yaw]
    :param map_params: 地图参数
    :param plot_enabled: 是否在搜索过程中绘制中间路径
    :return: (x_path, y_path, yaw_path, closed_set) 或 None
    """
    # 1. 初始化
    s_x, s_y, s_yaw = start_state
    g_x, g_y, g_yaw = goal_state

    start_grid_index = get_hybrid_grid_index(s_x, s_y, s_yaw)
    goal_grid_index = get_hybrid_grid_index(g_x, g_y, g_yaw)

    start_node = HybridAStarNode(
        start_grid_index, [start_state], 0.0, 1, 0.0, start_grid_index
    )
    goal_node = HybridAStarNode(
        goal_grid_index, [goal_state], 0.0, 1, 0.0, goal_grid_index
    )

    motion_commands = generate_motion_commands()

    # 2. 计算 Holonomic 启发式 (H-Cost)
    print("Calculating Holonomic Heuristics...")
    holonomic_heuristics = calculate_holonomic_cost_with_obstacles(
        goal_node, map_params
    )
    print("Holonomic Heuristics calculated.")

    def get_h_cost(x_idx: int, y_idx: int) -> float:
        """根据地图偏移获取 Holonomic 启发式成本"""
        rel_x = x_idx - map_params.map_min_x
        rel_y = y_idx - map_params.map_min_y
        if (
            rel_x < 0
            or rel_y < 0
            or rel_x >= holonomic_heuristics.shape[0]
            or rel_y >= holonomic_heuristics.shape[1]
        ):
            return float("inf")
        return holonomic_heuristics[rel_x, rel_y]

    # 3. Hybrid A* 搜索
    open_set: Dict[Tuple[int, int, int], HybridAStarNode] = {
        start_node.index: start_node
    }
    closed_set: Dict[Tuple[int, int, int], HybridAStarNode] = {}
    cost_queue = heapdict()

    # H = max(G_nonholonomic, H_holonomic * CostConfig.HYBRID_COST_WEIGHT)
    start_h_cost = get_h_cost(start_node.grid_index[0], start_node.grid_index[1])
    cost_queue[start_node.index] = (
        start_node.cost + CostConfig.HYBRID_COST_WEIGHT * start_h_cost
    )

    final_node: Optional[HybridAStarNode] = None

    while cost_queue:
        # 获取成本最低的节点
        current_index, f_cost = cost_queue.popitem()
        current_node = open_set.pop(current_index)
        closed_set[current_index] = current_node

        # 4. Reeds-Shepp 终点扩展 (找到 Reeds-Shepp 路径则结束)
        if current_index == goal_node.index or (current_node.cost + 200.0 < f_cost):
            # 尝试 Reeds-Shepp 扩展
            rs_node = reeds_shepp_expansion(current_node, goal_node, map_params)
            if rs_node:
                final_node = rs_node
                closed_set[rs_node.index] = rs_node
                print(f"Path Found via Reeds-Shepp! Cost: {rs_node.cost}")
                break

        # 5. 运动学扩展
        for motion_command in motion_commands:
            new_node = kinematic_simulation_node(
                current_node, motion_command, map_params
            )

            if new_node is None:
                continue

            # 绘制中间路径 (如果启用)
            if plot_enabled:
                x_traj, y_traj, _ = zip(*new_node.traj)
                plt.plot(x_traj, y_traj, linewidth=0.3, color="g")

            new_node_index = new_node.index

            if new_node_index in closed_set:
                continue

            # 计算 F-Cost
            h_cost_grid = get_h_cost(new_node.grid_index[0], new_node.grid_index[1])
            f_cost = new_node.cost + CostConfig.HYBRID_COST_WEIGHT * h_cost_grid

            if (
                new_node_index not in open_set
                or new_node.cost < open_set[new_node_index].cost
            ):
                open_set[new_node_index] = new_node
                cost_queue[new_node_index] = f_cost

    if not final_node:
        print("Path not found.")
        return None

    # 6. 回溯路径
    x_path, y_path, yaw_path = backtrack_path(start_node, final_node, closed_set)

    return x_path, y_path, yaw_path, closed_set


def backtrack_path(
    start_node: HybridAStarNode,
    final_node: HybridAStarNode,
    closed_set: Dict[Tuple[int, int, int], HybridAStarNode],
) -> Tuple[List[float], List[float], List[float]]:
    """从终点节点回溯到起点节点，构建最终路径"""

    current_node = final_node
    x, y, yaw = [], [], []

    while current_node.index != start_node.index:
        # 提取轨迹并反转，因为我们是从终点向起点回溯
        traj_x, traj_y, traj_yaw = zip(*current_node.traj)

        # 将轨迹段添加到总路径中
        x.extend(traj_x[::-1])
        y.extend(traj_y[::-1])
        yaw.extend(traj_yaw[::-1])

        if current_node.parent_index is None:
            # 路径中断，理论上不应该发生 (除非起点即终点)
            break

        current_node = closed_set.get(current_node.parent_index)
        if current_node is None:
            # 找不到父节点，路径中断
            print("Error: Backtrack path interrupted.")
            break

    # 因为是从后往前添加的，所以需要再次反转
    return x[::-1], y[::-1], yaw[::-1]
