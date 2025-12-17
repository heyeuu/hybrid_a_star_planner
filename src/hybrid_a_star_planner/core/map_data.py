import numpy as np
from scipy.spatial import KDTree
from typing import List, Tuple


class MapParameters:
    """地图参数和障碍物数据容器"""

    def __init__(
        self,
        map_min_x: int,
        map_min_y: int,
        map_max_x: int,
        map_max_y: int,
        xy_resolution: float,
        yaw_resolution: float,
        obstacle_kdtree: KDTree,
        obstacle_x: List[float],
        obstacle_y: List[float],
    ):
        self.map_min_x = map_min_x
        self.map_min_y = map_min_y
        self.map_max_x = map_max_x
        self.map_max_y = map_max_y
        self.xy_resolution = xy_resolution
        self.yaw_resolution = yaw_resolution
        self.obstacle_kdtree = obstacle_kdtree
        self.obstacle_x = obstacle_x
        self.obstacle_y = obstacle_y


def calculate_map_parameters(
    obstacle_x: List[float],
    obstacle_y: List[float],
    xy_resolution: float,
    yaw_resolution: float,
) -> MapParameters:
    """计算地图边界和生成障碍物 KD 树"""
    if not obstacle_x or not obstacle_y:
        # 处理空障碍物列表
        min_x, max_x = 0, 100
        min_y, max_y = 0, 100
    else:
        min_x, max_x = min(obstacle_x), max(obstacle_x)
        min_y, max_y = min(obstacle_y), max(obstacle_y)

    # 稍微扩展边界以确保包含所有点
    map_min_x = int(np.floor(min_x / xy_resolution))
    map_min_y = int(np.floor(min_y / xy_resolution))
    map_max_x = int(np.ceil(max_x / xy_resolution))
    map_max_y = int(np.ceil(max_y / xy_resolution))

    # 创建 KDTree 来表示障碍物
    if obstacle_x and obstacle_y:
        obstacle_kdtree = KDTree([[x, y] for x, y in zip(obstacle_x, obstacle_y)])
    else:
        # 如果没有障碍物，创建一个空的 KDTree
        obstacle_kdtree = KDTree([])

    return MapParameters(
        map_min_x,
        map_min_y,
        map_max_x,
        map_max_y,
        xy_resolution,
        yaw_resolution,
        obstacle_kdtree,
        obstacle_x,
        obstacle_y,
    )


def create_sample_map() -> Tuple[List[float], List[float]]:
    """生成示例地图障碍物坐标"""
    obstacle_x, obstacle_y = [], []

    # 边界
    for i in range(51):
        obstacle_x.extend([i, 0, i, 50])
        obstacle_y.extend([0, i, 50, i])

    # 内部障碍物
    for i in range(10, 20):
        obstacle_x.append(i)
        obstacle_y.append(30)
    for i in range(30, 51):
        obstacle_x.append(i)
        obstacle_y.append(30)
    for i in range(0, 31):
        obstacle_x.append(20)
        obstacle_y.append(i)
    for i in range(0, 31):
        obstacle_x.append(30)
        obstacle_y.append(i)
    for i in range(40, 50):
        obstacle_x.append(15)
        obstacle_y.append(i)
    for i in range(25, 40):
        obstacle_x.append(i)
        obstacle_y.append(35)

    return obstacle_x, obstacle_y
