import math


class CarConfig:
    """车辆运动学和尺寸参数"""

    MAX_STEER_ANGLE = 0.6  # 最大转向角 (rad)
    STEER_PRECISION = 10  # 转向角离散化数量 (单侧)
    WHEEL_BASE = 3.5  # 轴距 (m)
    AXLE_TO_FRONT = 4.5  # 后轴到前边缘 (m)
    AXLE_TO_BACK = 1.0  # 后轴到后边缘 (m)
    WIDTH = 3.0  # 车辆宽度 (m)


class AlgorithmConfig:
    """地图和算法通用参数"""

    XY_RESOLUTION = 4.0  # XY栅格分辨率 (m)
    YAW_RESOLUTION = math.radians(15.0)  # 航向角栅格分辨率 (rad)
    SIMULATION_LENGTH = 4.0  # 运动学模拟步长总长度 (m)
    SIMULATION_STEP = 0.8  # 运动学模拟中的小步长 (m)


class CostConfig:
    """路径成本权重"""

    REVERSE = 10.0  # 倒车惩罚
    DIRECTION_CHANGE = 150.0  # 换向惩罚
    STEER_ANGLE = 1.0  # 转向角大小惩罚
    STEER_ANGLE_CHANGE = 5.0  # 转向角变化惩罚
    HYBRID_COST_WEIGHT = 50.0  # Holonomic 启发式权重
