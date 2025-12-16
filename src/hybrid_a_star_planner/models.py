class Car:
    max_steer_angle = 0.6
    steer_presion = 10
    wheel_base = 3.5
    axle_to_front = 4.5
    axle_to_back = 1
    width = 3


class Cost:
    reverse = 10
    direction_change = 150
    steering_angle = 1
    steering_angle_change = 5
    hybrid_cost = 50


class Node:
    def __init__(
        self, grid_index, trajectory, steering_angle, direction, cost, parent_index
    ) -> None:
        self.grid_index = grid_index
        self.trajectory = trajectory
        self.steering_angle = steering_angle
        self.direction = direction
        self.cost = cost
        self.parent_index = parent_index


class MapParameters:
    def __init__(
        self,
        map_min_x,
        map_max_x,
        map_min_y,
        map_max_y,
        xy_resolution,
        yaw_resolution,
        obstacle_KDTree,
        obstacle_x,
        obstacle_y,
    ) -> None:
        self.mapMinX = map_min_x
        self.mapMaxX = map_max_x
        self.mapMinY = map_min_y
        self.mapMaxY = map_max_y
        self.xyResolution = xy_resolution
        self.yawResolution = yaw_resolution
        self.obstacleKDTree = obstacle_KDTree
        self.obstacleX = obstacle_x
        self.obstacleY = obstacle_y


class HolonomicNode:
    def __init__(self, grid_index, cost, parent_index) -> None:
        self.gridIndex = grid_index
        self.cost = cost
        self.parentIndex = parent_index
