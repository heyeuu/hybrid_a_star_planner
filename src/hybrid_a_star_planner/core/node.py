from typing import List, Tuple, Optional


class HybridAStarNode:
    """Hybrid A* 算法中的非完整节点（包含车辆状态）"""

    def __init__(
        self,
        grid_index: Tuple[int, int, int],
        traj: List[List[float]],
        steering_angle: float,
        direction: int,
        cost: float,
        parent_index: Optional[Tuple[int, int, int]],
    ):
        self.grid_index: Tuple[int, int, int] = grid_index  # (x_idx, y_idx, yaw_idx)
        self.traj: List[List[float]] = traj  # [(x, y, yaw), ...]
        self.steering_angle: float = steering_angle  # 轨迹上的转向角
        self.direction: int = direction  # 轨迹上的方向 (+1 前进, -1 后退)
        self.cost: float = cost  # G-Cost
        self.parent_index: Optional[Tuple[int, int, int]] = parent_index  # 父节点索引

    @property
    def index(self) -> Tuple[int, int, int]:
        """获取用于哈希或查找的节点索引"""
        return self.grid_index

    @property
    def x(self) -> float:
        return self.traj[-1][0]

    @property
    def y(self) -> float:
        return self.traj[-1][1]

    @property
    def yaw(self) -> float:
        return self.traj[-1][2]


class HolonomicNode:
    """Holonomic A* 算法中的完整节点（仅有XY位置）"""

    def __init__(
        self, grid_index: Tuple[int, int], cost: float, parent_index: Tuple[int, int]
    ):
        self.grid_index: Tuple[int, int] = grid_index  # (x_idx, y_idx)
        self.cost: float = cost
        self.parent_index: Tuple[int, int] = parent_index

    @property
    def index(self) -> Tuple[int, int]:
        """获取用于哈希或查找的节点索引"""
        return self.grid_index
