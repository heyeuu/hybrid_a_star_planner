import math
from typing import List, Tuple
from ..config import CarConfig
from ..core.map_data import MapParameters,DynamicObstacle


def is_within_map_bounds(
    grid_index: Tuple[int, int, int], map_params: MapParameters
) -> bool:
    """检查节点是否在地图边界内"""
    x_idx, y_idx, _ = grid_index
    if (
        x_idx <= map_params.map_min_x
        or x_idx >= map_params.map_max_x
        or y_idx <= map_params.map_min_y
        or y_idx >= map_params.map_max_y
    ):
        return False
    return True


def check_collision(traj: List[List[float]], map_params: MapParameters) -> bool:
    """
    检查轨迹上的车辆是否与障碍物发生碰撞。
    使用膨胀的圆形和车辆的精确尺寸模型结合 KDTree。
    """
    # 车辆中心到参考点（后轴中心）的距离
    dl = (CarConfig.AXLE_TO_FRONT - CarConfig.AXLE_TO_BACK) / 2

    # 碰撞检测半径：取车辆半长
    car_radius = (CarConfig.AXLE_TO_FRONT + CarConfig.AXLE_TO_BACK) / 2

    for x, y, yaw in traj:
        # 车辆几何中心
        cx = x + dl * math.cos(yaw)
        cy = y + dl * math.sin(yaw)

        # 1. 粗略检查：找到在车辆几何中心附近半径内的障碍物点
        # 搜索半径稍微大一点，以保证找到所有相关的障碍物点
        points_in_obstacle_indices = map_params.obstacle_kdtree.query_ball_point(
            [cx, cy], car_radius + 1.0
        )

        if not points_in_obstacle_indices:
            continue

        # 2. 精确检查：对于找到的每个障碍物点，检查其是否在车辆的包围盒内
        for p_idx in points_in_obstacle_indices:
            xo = map_params.obstacle_x[p_idx] - cx
            yo = map_params.obstacle_y[p_idx] - cy

            # 将障碍物点坐标 (xo, yo) 旋转到车辆坐标系
            dx = xo * math.cos(yaw) + yo * math.sin(yaw)
            dy = -xo * math.sin(yaw) + yo * math.cos(yaw)

            # 检查障碍物点是否落入车辆的近似矩形包围盒内
            # 这里的 car_radius 用作车辆的半长（x方向），CarConfig.WIDTH / 2 是半宽（y方向）
            if abs(dx) < car_radius and abs(dy) < CarConfig.WIDTH / 2:
                return True  # 发生碰撞

    return False

def is_collision_with_dynamic_obstacles(
    traj: List[tuple],
    map_params: MapParameters,
    start_time: float = 0.0,
    dt: float = 0.2,
    threshold: float = 2.0  #阈值越大，越容易判断为碰撞，离障碍物远远的，这里根据栅格大小以及车辆的大小设为栅格的1/2
) -> bool:
    """
    检查轨迹是否与动态障碍物发生时空碰撞
    :param traj: 节点轨迹 [(x, y, yaw), ...]
    :param map_params: 地图参数，包含动态障碍物
    :param start_time: 轨迹起始时间
    :param dt: 每个轨迹点的时间间隔
    :param threshold: 判定碰撞的距离阈值
    :return: True-发生碰撞，False-无碰撞
    """
    for i, (x, y, _) in enumerate(traj):
        t = start_time + i * dt
        for obs in getattr(map_params, "dynamic_obstacles", []):
            obs_x, obs_y = obs.get_position(t)
            dist_sq = (x - obs_x) ** 2 + (y - obs_y) ** 2
            if dist_sq < threshold ** 2:
                return True
    return False

def is_valid(
    traj: List[List[float]], grid_index: Tuple[int, int, int], map_params: MapParameters,start_time: float = 0.0, dt: float = 1.0, threshold: float = 2.0
) -> bool:
    """检查节点是否有效：在地图边界内且无碰撞,（含动态障碍物）"""
    if not is_within_map_bounds(grid_index, map_params):
        return False

    if check_collision(traj, map_params):
        return False

    # 动态障碍物碰撞检测
    if is_collision_with_dynamic_obstacles(traj, map_params, start_time, dt, threshold):
        return False
    
    return True
