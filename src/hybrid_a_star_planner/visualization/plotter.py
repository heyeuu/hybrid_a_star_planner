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


def plot_final_path(
    x_path: List[float],
    y_path: List[float],
    yaw_path: List[float],
    map_params: MapParameters,
    title: str = "Hybrid A* Path Planning",
):
    """绘制最终路径和动画效果"""
    plt.figure(figsize=(10, 10))

    # 绘制障碍物
    plt.plot(map_params.obstacle_x, map_params.obstacle_y, "sk", label="Obstacles")

    # 设置图表限制
    plt.xlim(min(map_params.obstacle_x) - 5, max(map_params.obstacle_x) + 5)
    plt.ylim(min(map_params.obstacle_y) - 5, max(map_params.obstacle_y) + 5)
    plt.title(title)
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.grid(True)

    # 绘制完整路径
    plt.plot(x_path, y_path, linewidth=1.5, color="r", zorder=0, label="Final Path")

    # 动画展示车辆行驶
    for k in range(0, len(x_path), 5):  # 每隔5个点绘制一次，减少绘图开销
        plt.cla()
        plt.plot(map_params.obstacle_x, map_params.obstacle_y, "sk")
        plt.plot(x_path, y_path, linewidth=1.5, color="r", zorder=0)

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

        plt.pause(0.001)

    plt.show()
