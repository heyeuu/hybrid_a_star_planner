import math
import heapq
import numpy as np
from typing import List, Tuple, Dict
from ..core.node import HolonomicNode, HybridAStarNode
from ..core.map_data import MapParameters
from ..core.util import get_holonomic_grid_index


def euclidean_cost(motion_command: Tuple[int, int]) -> float:
    """计算 Holonomic A* 中的欧几里得成本"""
    return math.hypot(motion_command[0], motion_command[1])


def get_holonomic_motion_commands() -> List[Tuple[int, int]]:
    return [(-1, 0), (-1, 1), (0, 1), (1, 1), (1, 0), (1, -1), (0, -1), (-1, -1)]


def create_holonomic_obstacle_map(map_params: MapParameters) -> np.ndarray:
    """
    根据障碍物坐标和分辨率创建 Holonomic A* 的栅格地图。
    地图边界由 map_params 决定。
    """
    # 计算地图的尺寸
    max_x_idx = map_params.map_max_x
    max_y_idx = map_params.map_max_y

    # 初始化为 False (无障碍物)
    obstacle_map = np.full((max_x_idx + 1, max_y_idx + 1), False)

    # 将障碍物点映射到栅格并标记为 True
    for obs_x, obs_y in zip(map_params.obstacle_x, map_params.obstacle_y):
        x_idx = int(round(obs_x / map_params.xy_resolution))
        y_idx = int(round(obs_y / map_params.xy_resolution))

        # 确保索引在范围内
        if 0 <= x_idx <= max_x_idx and 0 <= y_idx <= max_y_idx:
            # 简单地将障碍物点所在的栅格标记为障碍物
            obstacle_map[x_idx, y_idx] = True

    return obstacle_map


def is_holonomic_node_valid(
    node: HolonomicNode, obstacle_map: np.ndarray, map_params: MapParameters
) -> bool:
    """检查 Holonomic A* 节点是否有效 (边界内且不在障碍物上)"""
    x_idx, y_idx = node.grid_index

    # 检查地图边界
    if (
        x_idx < map_params.map_min_x
        or x_idx > map_params.map_max_x
        or y_idx < map_params.map_min_y
        or y_idx > map_params.map_max_y
    ):
        return False

    # 检查障碍物,还需要处理障碍物地图的索引范围
    map_max_x_idx, map_max_y_idx = obstacle_map.shape
    if 0 <= x_idx < map_max_x_idx and 0 <= y_idx < map_max_y_idx:
        if obstacle_map[x_idx, y_idx]:
            return False
    else:
        # 索引超出障碍物地图的实际计算范围
        return False

    return True


def calculate_holonomic_cost_with_obstacles(
    goal_node: HybridAStarNode, map_params: MapParameters
) -> np.ndarray:
    """
    使用 Dijkstra/A* (反向搜索) 计算 Holonomic 启发式成本矩阵。
    结果是每个 (x, y) 栅格到终点的最短路径成本。
    """
    # 1. 初始化
    goal_x_idx, goal_y_idx = get_holonomic_grid_index(goal_node.x, goal_node.y)
    start_node = HolonomicNode((goal_x_idx, goal_y_idx), 0.0, (goal_x_idx, goal_y_idx))

    # 2. 创建障碍物地图和动作集
    obstacle_map = create_holonomic_obstacle_map(map_params)
    motion_commands = get_holonomic_motion_commands()

    # 3. 反向搜索
    open_set: Dict[Tuple[int, int], HolonomicNode] = {start_node.index: start_node}
    closed_set: Dict[Tuple[int, int], HolonomicNode] = {}
    priority_queue: List[Tuple[float, Tuple[int, int]]] = [
        (0.0, start_node.index)
    ]  # (cost, index)

    while priority_queue:
        cost, current_index = heapq.heappop(priority_queue)
        current_node = open_set.pop(current_index)

        if current_index in closed_set:
            continue

        closed_set[current_index] = current_node

        for dx, dy in motion_commands:
            neighbor_index = (
                current_node.grid_index[0] + dx,
                current_node.grid_index[1] + dy,
            )
            cost_increment = euclidean_cost((dx, dy))
            neighbor_cost = current_node.cost + cost_increment

            neighbor_node = HolonomicNode(neighbor_index, neighbor_cost, current_index)

            if not is_holonomic_node_valid(neighbor_node, obstacle_map, map_params):
                continue

            if neighbor_index in closed_set:
                continue

            if (
                neighbor_index not in open_set
                or neighbor_cost < open_set[neighbor_index].cost
            ):
                open_set[neighbor_index] = neighbor_node
                heapq.heappush(priority_queue, (neighbor_cost, neighbor_index))

    # 4. 提取启发式成本矩阵
    max_x_idx, max_y_idx = obstacle_map.shape
    holonomic_cost_matrix = np.full((max_x_idx, max_y_idx), np.inf)

    for node in closed_set.values():
        x_idx, y_idx = node.grid_index
        if 0 <= x_idx < max_x_idx and 0 <= y_idx < max_y_idx:
            holonomic_cost_matrix[x_idx, y_idx] = node.cost

    return holonomic_cost_matrix
