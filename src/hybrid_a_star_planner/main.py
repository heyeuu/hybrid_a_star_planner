import numpy as np

from .config import AlgorithmConfig
from .core.map_data import calculate_map_parameters, create_sample_map,DynamicObstacle
from .path_planner.hybrid_a_star import run_hybrid_a_star
from .visualization.plotter import plot_final_path
from .visualization.plotter import plot_dynamic_obstacles

def main():
    # 1. 设置起点和终点
    start_state = [15.0, 10.0, np.deg2rad(0.0)]
    goal_state = [45.0, 35.0, np.deg2rad(90.0)]

    # 2. 获取障碍物地图数据
    obstacle_x, obstacle_y = create_sample_map()

    # 3. 计算地图参数
    map_params = calculate_map_parameters(
        obstacle_x,
        obstacle_y,
        AlgorithmConfig.XY_RESOLUTION,
        AlgorithmConfig.YAW_RESOLUTION,
    )

    # 3.1 添加动态障碍物
    def traj_func(t):   #这个t会在后面传入
    # 在y=33~39之间来回移动，x=22,设置在原先仅有静态障碍物时的路径必经点
        y = 36 + 3 * np.sin(0.2* t)  
        x = 22
        return (x, y)

    dynamic_obstacle1 = DynamicObstacle(
        init_pos=(22, 36),
        velocity=(0, 0),
        trajectory_func=traj_func
    )
    map_params.dynamic_obstacles.append(dynamic_obstacle1)

    # 4. 运行 Hybrid A*
    # 启用 plot_enabled=True 可以看到搜索过程中的中间路径
    result = run_hybrid_a_star(start_state, goal_state, map_params, plot_enabled=False)

    if result:
        x_path, y_path, yaw_path, _closed_set = result

        # 5. 可视化最终结果
        # plot_dynamic_obstacles(map_params.dynamic_obstacles, total_time=50, dt=1.0)
        plot_final_path(x_path, y_path, yaw_path, map_params)
    else:
        print("Path planning failed to find a path.")


if __name__ == "__main__":
    try:
        main()
    except ModuleNotFoundError as e:
        print(f"Error: {e}")
        print(
            "Please ensure you are running 'main.py' from the 'hybrid_a_star' project root directory."
        )
