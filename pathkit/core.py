"""pathkit 核心 API（初始未实现）。"""


class PathBuilder:
    def __init__(self):
        """构造空路径"""
        raise NotImplementedError

    def move_to(self, x, y):
        """移动到起点"""
        raise NotImplementedError

    def line_to(self, x, y):
        """直线段"""
        raise NotImplementedError

    def curve_to(self, c1, c2, end):
        """三次曲线（3 个点）"""
        raise NotImplementedError

    def close(self):
        """闭合"""
        raise NotImplementedError

    def flatten(self, tolerance):
        """自适应扁平化"""
        raise NotImplementedError

    def arc_length(self):
        """路径总弧长"""
        raise NotImplementedError

    def split_at_length(self, distance):
        """按弧长切分，返回 (前半, 后半)"""
        raise NotImplementedError

    def snapshot(self):
        """快照"""
        raise NotImplementedError

    def restore(self, marker):
        """回滚"""
        raise NotImplementedError

    def bounds(self):
        """包围盒"""
        raise NotImplementedError

    def point_count(self):
        """顶点数"""
        raise NotImplementedError
