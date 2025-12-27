# Hybrid A* Planner

本项目实现了面向车辆的 Hybrid A* 路径规划器，旨在生成满足非完整约束
（最小转弯半径、前进/倒车切换）的可行路径。算法融合了网格搜索的全局性
与运动学模型的可行性，并采用 Reeds-Shepp 曲线在终点附近完成路径闭合。

## 项目目标

- 在栅格地图中求解起点至终点的可行车辆轨迹
- 轨迹严格遵循自行车模型与转向约束
- 在障碍环境下提供可视化结果以便分析

## 算法组成

1) Hybrid A*（主搜索）
- 采用离散转向角与前进/倒车动作集扩展节点
- 基于运动学仿真生成局部轨迹
- 使用启发式代价引导搜索收敛

2) Holonomic A*（启发式）
- 在二维栅格上执行反向搜索
- 为每个 (x, y) 位置计算到终点的估计成本

3) Reeds-Shepp 终点扩展
- 基于最大曲率生成最短可行曲线
- 若无碰撞则直接拼接为最终路径

## 效果与优势

- 输出满足车辆转向约束的可行路径
- 在复杂障碍环境中具备较好的可达性
- 结构清晰，便于替换车辆模型与参数
- Reeds-Shepp 终点扩展提升收敛效率

## demo
<div align="center">
  <video src="https://work.heyeuuu19.com/course/AI/demo_720p.mp4" muted autoplay loop playsinline width="100%"></video>
</div>
## 学习路径

建议按以下顺序阅读源码：

1) `src/hybrid_a_star_planner/main.py`
- 入口示例，包含地图生成与主流程

2) `src/hybrid_a_star_planner/path_planner/hybrid_a_star.py`
- Hybrid A* 主循环与代价构成

3) `src/hybrid_a_star_planner/path_planner/kinematics.py`
- 运动学模型与动作集离散化

4) `src/hybrid_a_star_planner/path_planner/reeds_shepp_path.py`
- Reeds-Shepp 终点扩展实现

## 使用说明

### 环境要求

- Python >= 3.13
- Poetry

### 安装

```bash
poetry install
```

### 运行

```bash
poetry run python -m hybrid_a_star_planner.main
```

如在无 GUI 环境运行，建议将可视化改为保存图片，以避免
`plt.show()` 阻塞或后端警告。

## 项目结构

```
src/hybrid_a_star_planner/
  core/                 # 数据结构与工具
  path_planner/         # Hybrid A* / Holonomic A* / Reeds-Shepp
  visualization/        # 可视化与动画
  config.py             # 车辆与算法参数
  main.py               # 入口示例
```

## 关键参数

参数集中于 `src/hybrid_a_star_planner/config.py`：

- `CarConfig.MAX_STEER_ANGLE` 最大转向角
- `CarConfig.WHEEL_BASE` 轴距
- `AlgorithmConfig.XY_RESOLUTION` 栅格分辨率
- `AlgorithmConfig.YAW_RESOLUTION` 航向角分辨率
- `AlgorithmConfig.SIMULATION_LENGTH` 单步仿真长度
- `AlgorithmConfig.SIMULATION_STEP` 仿真步长