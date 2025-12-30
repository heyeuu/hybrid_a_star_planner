import math
import numpy as np
import matplotlib

matplotlib.use("qtagg")
import matplotlib.pyplot as plt
from typing import List
from ..config import CarConfig
from ..core.map_data import MapParameters


def draw_car(x: float, y: float, yaw: float, color: str = "black", ax=None):
    """在给定位置和方向绘制车辆轮廓"""
    if ax is None:
        ax = plt.gca()

    # 车辆轮廓 (相对于后轴中心)
    car = np.array(
        [
            [
                -CarConfig.AXLE_TO_BACK,
                -CarConfig.AXLE_TO_BACK,
                CarConfig.AXLE_TO_FRONT,
                CarConfig.AXLE_TO_FRONT,
                -CarConfig.AXLE_TO_BACK,
            ],
            [
                CarConfig.WIDTH / 2,
                -CarConfig.WIDTH / 2,
                -CarConfig.WIDTH / 2,
                CarConfig.WIDTH / 2,
                CarConfig.WIDTH / 2,
            ],
        ]
    )

    # 旋转矩阵
    rotation_z = np.array(
        [[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]]
    )

    # 旋转和位移
    car = np.dot(rotation_z, car)
    car += np.array([[x], [y]])

    ax.plot(car[0, :], car[1, :], color)

def plot_dynamic_obstacles(dynamic_obstacles, total_time, dt=1.0, color='r', label_prefix='DynObs'):
    """
    绘制动态障碍物的运动轨迹
    :param dynamic_obstacles: 动态障碍物列表
    :param total_time: 轨迹总时长
    :param dt: 采样时间间隔
    :param color: 轨迹颜色
    :param label_prefix: 图例前缀
    """
    t_list = np.arange(0, total_time, dt)
    for idx, obs in enumerate(dynamic_obstacles):
        traj_x, traj_y = [], []
        for t in t_list:
            x, y = obs.get_position(t)
            traj_x.append(x)
            traj_y.append(y)
        plt.plot(traj_x, traj_y, color=color, linestyle='--', linewidth=1.2, label=f"{label_prefix}{idx+1}")
        # 可选：绘制障碍物当前位置
        plt.scatter(traj_x[-1], traj_y[-1], color=color, marker='o')

    # 避免重复图例
    handles, labels = plt.gca().get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    plt.legend(by_label.values(), by_label.keys())


def plot_final_path(
    x_path: List[float],
    y_path: List[float],
    yaw_path: List[float],
    map_params: MapParameters,
    title: str = "Hybrid A* Path Planning",
):
    plt.figure(figsize=(10, 10))

    # 预先计算障碍物轨迹长度
    path_len = len(x_path)
    t_list = np.arange(0, path_len, 1)

    for k in range(0, path_len, 1):  # 每个点都绘制，动画更流畅
        plt.cla()
        plt.plot(map_params.obstacle_x, map_params.obstacle_y, "sk", label="Obstacles")

        # 动态障碍物当前位置
        if hasattr(map_params, "dynamic_obstacles") and map_params.dynamic_obstacles:
            for idx, obs in enumerate(map_params.dynamic_obstacles):
                # 只显示当前位置
                x_dyn, y_dyn = obs.get_position(t_list[k])
                plt.scatter(x_dyn, y_dyn, color="b", marker="o", label=f"DynamicObs{idx+1}" if k == 0 else "")

        # 绘制已生成的路径
        plt.plot(x_path[:k+1], y_path[:k+1], linewidth=1.5, color="r", zorder=0, label="Final Path" if k == 0 else "")

        draw_car(x_path[k], y_path[k], yaw_path[k])

        # 绘制车辆方向箭头
        plt.arrow(
            x_path[k],
            y_path[k],
            -math.cos(yaw_path[k]),
            -math.sin(yaw_path[k]),
            width=0.1,
            head_width=0.3,
            color="b",
        )

        plt.xlim(min(map_params.obstacle_x) - 5, max(map_params.obstacle_x) + 5)
        plt.ylim(min(map_params.obstacle_y) - 5, max(map_params.obstacle_y) + 5)
        plt.title(f"{title} (Step {k})")

        # 避免重复图例
        handles, labels = plt.gca().get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        if k == 0:
            plt.legend(by_label.values(), by_label.keys())

        plt.pause(0.02)

    plt.show()
