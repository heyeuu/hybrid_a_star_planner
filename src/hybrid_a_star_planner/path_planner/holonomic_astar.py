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
    # 使用边界范围而不是绝对坐标大小，避免创建过大的稠密矩阵
    width = map_params.map_max_x - map_params.map_min_x + 1
    height = map_params.map_max_y - map_params.map_min_y + 1
    obstacle_map = np.full((width, height), False)

    # 将障碍物点映射到栅格并标记为 True
    for obs_x, obs_y in zip(map_params.obstacle_x, map_params.obstacle_y):
        x_idx = int(round(obs_x / map_params.xy_resolution))
        y_idx = int(round(obs_y / map_params.xy_resolution))

        # 将绝对索引转换为相对索引以写入数组
        rel_x = x_idx - map_params.map_min_x
        rel_y = y_idx - map_params.map_min_y

        # 确保索引在范围内
        if 0 <= rel_x < width and 0 <= rel_y < height:
            obstacle_map[rel_x, rel_y] = True

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

    # 计算相对索引以访问障碍物矩阵
    rel_x = x_idx - map_params.map_min_x
    rel_y = y_idx - map_params.map_min_y
    map_width, map_height = obstacle_map.shape

    if 0 <= rel_x < map_width and 0 <= rel_y < map_height:
        if obstacle_map[rel_x, rel_y]:
            return False
    else:
        # 超出矩阵范围
        return False

    return True


def calculate_holonomic_cost_with_obstacles(
    goal_node: HybridAStarNode, map_params: MapParameters
) -> np.ndarray:
    """
    使用 Dijkstra (反向搜索) 计算 Holonomic 启发式成本矩阵。
    结果是每个 (x, y) 栅格到终点的最短路径成本。
    """
    # 1. 初始化
    goal_x_idx, goal_y_idx = get_holonomic_grid_index(
        goal_node.x, goal_node.y, map_params.xy_resolution
    )
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
        current_node = open_set.pop(current_index, None)
        if current_node is None or current_index in closed_set:
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
    map_width, map_height = obstacle_map.shape
    holonomic_cost_matrix = np.full((map_width, map_height), np.inf)

    for node in closed_set.values():
        x_idx, y_idx = node.grid_index
        rel_x = x_idx - map_params.map_min_x
        rel_y = y_idx - map_params.map_min_y

        if 0 <= rel_x < map_width and 0 <= rel_y < map_height:
            holonomic_cost_matrix[rel_x, rel_y] = node.cost

    return holonomic_cost_matrix
