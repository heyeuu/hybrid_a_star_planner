import numpy as np

from .config import AlgorithmConfig
from .core.map_data import calculate_map_parameters, create_sample_map
from .path_planner.hybrid_a_star import run_hybrid_a_star
from .visualization.plotter import plot_final_path


def main():
    # 1. 设置起点和终点
    start_state = [15.0, 10.0, np.deg2rad(90.0)]
    goal_state = [43.0, 35.0, np.deg2rad(0.0)]

    # 2. 获取障碍物地图数据
    obstacle_x, obstacle_y = create_sample_map()

    # 3. 计算地图参数
    map_params = calculate_map_parameters(
        obstacle_x,
        obstacle_y,
        AlgorithmConfig.XY_RESOLUTION,
        AlgorithmConfig.YAW_RESOLUTION,
    )

    # 4. 运行 Hybrid A*
    # 启用 plot_enabled=True 可以看到搜索过程中的中间路径
    result = run_hybrid_a_star(start_state, goal_state, map_params, plot_enabled=True)

    if result:
        x_path, y_path, yaw_path, closed_set = result

        # 5. 可视化最终结果
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
