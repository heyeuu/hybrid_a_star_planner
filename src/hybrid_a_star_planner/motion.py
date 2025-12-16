import math
import numpy as np
from models import Car, Cost, Node


def motion_command():
    step = Car.max_steer_angle / Car.steer_presion
    angles = np.arange(Car.max_steer_angle, -Car.max_steer_angle - step, -step)
    directions = np.array([1, -1])
    return np.array(
        [[angle, direction] for angle in angles for direction in directions]
    )


def holonomicMotionCommand():
    grid = np.array([(dx, dy) for dx in (-1, 0, 1) for dy in (-1, 0, 1)])
    return grid[(grid != (0, 0)).any(axis=1)]


def pi_2_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def collision(trajectory, map_parameters, safe_margin=1) -> bool:
    half_length = (Car.axle_to_front + Car.axle_to_back) / 2 + safe_margin
    search_radius = half_length + safe_margin
    center_offset_dl = (Car.axle_to_front - Car.axle_to_back) / 2
    half_safe_width = Car.width / 2 + safe_margin

    for x, y, yaw in trajectory:
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        center_x = x + center_offset_dl * cos_yaw
        center_y = y + center_offset_dl * sin_yaw
        obstacle_indices = map_parameters.obstacle_KDTree.query_ball_point(
            [center_x, center_y], search_radius
        )
        if not obstacle_indices:
            continue

        obs_x = map_parameters.obstacle_x[obstacle_indices]
        obs_y = map_parameters.obstacle_y[obstacle_indices]

        relative_x = obs_x - center_x
        relative_y = obs_y - center_y

        local_dx = relative_x * cos_yaw + relative_y * sin_yaw
        local_dy = -relative_x * sin_yaw + relative_y * cos_yaw

        is_collision = np.any(
            (np.abs(local_dx) < half_length) & (np.abs(local_dy) < half_safe_width)
        )
        if is_collision:
            return True
    return False


def is_valid(traj, grid_index, map_parameters):
    ix, iy, _ = grid_index
    if not (map_parameters.mapMinX < ix < map_parameters.mapMaxX):
        return False
    if not (map_parameters.mapMinY < iy < map_parameters.mapMaxY):
        return False

    if collision(traj, map_parameters):
        return False

    return True


def simulate_path_cost(current_node, motion_command, simulation_length) -> float:
    new_steering_angle, new_direction = motion_command
    total_cost = current_node.cost

    distance_cost_multiplier = Cost.reverse if new_direction != 1 else 1.0
    total_cost = simulation_length * distance_cost_multiplier

    if current_node.direction != new_direction:
        total_cost += Cost.direction_change

    total_cost += abs(new_steering_angle) * Cost.steering_angle

    steering_angle_change = abs(new_steering_angle - current_node.steering_angle)
    total_cost += steering_angle_change * Cost.steering_angle_change
    return total_cost


def index(Node):
    return tuple([Node.grid_index[0], Node.grid_index[1], Node.grid_index[2]])


def kinematicSimulationNode(
    current_node, motion_command, map_parameters, simulation_length=4, step=0.8
):
    steering_angle, direction = motion_command()
    x, y, yaw = current_node.trajectory[-1]

    num_steps = int(simulation_length / step)
    traj = []
    for _ in range(num_steps):
        dyaw = direction * step / Car.wheel_base * math.tan(steering_angle)
        yaw_new = pi_2_pi(yaw + dyaw)
        dx = direction * step * math.cos(yaw_new)
        dy = direction * step * math.sin(yaw_new)

        x_new = x + dx
        y_new = y + dy

        x, y, yaw = x_new, y_new, yaw_new
        traj.append([x, y, yaw])

    grid_index = [
        round(traj[-1][0] / map_parameters.xyResolution),
        round(traj[-1][1] / map_parameters.xyResolution),
        round(traj[-1][2] / map_parameters.yawResolution),
    ]

    if not is_valid(traj, grid_index, map_parameters):
        return None

    cost = simulate_path_cost(current_node, motion_command, simulation_length)

    return Node(grid_index, traj, steering_angle, direction, cost, index(current_node))
