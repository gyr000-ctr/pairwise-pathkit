"""pathkit 核心 API。

路径由 move/line/cubic-curve/close 操作序列构成。
- flatten: 自适应细分（de Casteljau），保证折线与真实曲线最大偏差不超过容差
- arc_length: Gauss-Legendre 数值积分，高精度弧长
- split_at_length: 在曲线上精确切分（de Casteljau 分割，切点严格落在曲线上）
- snapshot/restore: 操作序列级快照，回放一致
"""

import math

__all__ = ["PathBuilder"]

_MAX_POINTS = 10000
_MAX_DEPTH = 40

# 10 点 Gauss-Legendre 求积节点与权重（区间 [-1, 1]）
_GL_X = (
    -0.9739065285171717, -0.8650633666889845, -0.6794095682990244,
    -0.4333953941292472, -0.14887433898163122, 0.14887433898163122,
    0.4333953941292472, 0.6794095682990244, 0.8650633666889845,
    0.9739065285171717,
)
_GL_W = (
    0.06667134430868814, 0.1494513491505806, 0.21908636251598204,
    0.26926671930999635, 0.29552422471475287, 0.29552422471475287,
    0.26926671930999635, 0.21908636251598204, 0.1494513491505806,
    0.06667134430868814,
)


def _dist(a, b):
    return math.hypot(b[0] - a[0], b[1] - a[1])


def _lerp(a, b, t):
    return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)


def _pt_line_dist(p, a, b):
    """点 p 到直线 ab 的距离（ab 退化时为点到点距离）。"""
    dx, dy = b[0] - a[0], b[1] - a[1]
    if dx == 0 and dy == 0:
        return _dist(p, a)
    return abs(dy * (p[0] - a[0]) - dx * (p[1] - a[1])) / math.hypot(dx, dy)


def _cubic_speed(p0, p1, p2, p3, t):
    mt = 1.0 - t
    dx = (3.0 * mt * mt * (p1[0] - p0[0])
          + 6.0 * mt * t * (p2[0] - p1[0])
          + 3.0 * t * t * (p3[0] - p2[0]))
    dy = (3.0 * mt * mt * (p1[1] - p0[1])
          + 6.0 * mt * t * (p2[1] - p1[1])
          + 3.0 * t * t * (p3[1] - p2[1]))
    return math.hypot(dx, dy)


def _cubic_length(p0, p1, p2, p3, t0=0.0, t1=1.0):
    """三次贝塞尔在 [t0, t1] 上的弧长（Gauss-Legendre 积分）。"""
    mid = 0.5 * (t0 + t1)
    half = 0.5 * (t1 - t0)
    s = 0.0
    for x, w in zip(_GL_X, _GL_W):
        s += w * _cubic_speed(p0, p1, p2, p3, mid + half * x)
    return s * half


def _split_cubic(p0, p1, p2, p3, t):
    """de Casteljau 在参数 t 处分割，返回 (前三元组控制点, 后三元组控制点, 切点)。"""
    p01 = _lerp(p0, p1, t)
    p12 = _lerp(p1, p2, t)
    p23 = _lerp(p2, p3, t)
    p012 = _lerp(p01, p12, t)
    p123 = _lerp(p12, p23, t)
    p0123 = _lerp(p012, p123, t)
    return (p0, p01, p012, p0123), (p0123, p123, p23, p3), p0123


def _flatten_cubic(p0, p1, p2, p3, tol, out, depth):
    # 控制点到弦的距离是曲线上界的 4/3 倍，留 0.75 安全系数后仍保证 <= tol
    if depth >= _MAX_DEPTH or max(
        _pt_line_dist(p1, p0, p3), _pt_line_dist(p2, p0, p3)
    ) <= tol:
        out.append(p3)
        return
    first, second, _ = _split_cubic(p0, p1, p2, p3, 0.5)
    _flatten_cubic(*first, tol, out, depth + 1)
    _flatten_cubic(*second, tol, out, depth + 1)


class PathBuilder:
    def __init__(self):
        """构造空路径"""
        self._ops = []
        self._cur = None
        self._start = None
        self._npoints = 0
        self._snapshots = {}
        self._next_token = 0

    # ---- 构建 ----

    def _reserve(self, n):
        if self._npoints + n > _MAX_POINTS:
            raise ValueError("point limit %d exceeded" % _MAX_POINTS)

    def move_to(self, x, y):
        """移动到起点"""
        self._reserve(1)
        p = (x, y)
        self._ops.append(("move", p))
        self._cur = self._start = p
        self._npoints += 1
        return self

    def line_to(self, x, y):
        """直线段"""
        if self._cur is None:
            raise ValueError("line_to before move_to")
        self._reserve(1)
        p = (x, y)
        self._ops.append(("line", p))
        self._cur = p
        self._npoints += 1
        return self

    def curve_to(self, *points):
        """三次曲线（3 个点：c1, c2, end）"""
        if len(points) != 3:
            raise ValueError("curve_to requires exactly 3 points")
        if self._cur is None:
            raise ValueError("curve_to before move_to")
        self._reserve(3)
        c1, c2, end = (tuple(p) for p in points)
        self._ops.append(("curve", c1, c2, end))
        self._cur = end
        self._npoints += 3
        return self

    def close(self):
        """闭合当前子路径"""
        if self._cur is None:
            raise ValueError("nothing to close")
        self._ops.append(("close",))
        self._cur = self._start
        return self

    # ---- 测量 ----

    def flatten(self, tolerance):
        """自适应扁平化：折线与真实曲线最大偏差不超过 tolerance"""
        if tolerance is None or tolerance <= 0:
            raise ValueError("tolerance must be positive")
        flat_tol = 0.75 * tolerance
        pts = []
        cur = start = None
        for op in self._ops:
            kind = op[0]
            if kind == "move":
                cur = start = op[1]
                pts.append(cur)
            elif kind == "line":
                cur = op[1]
                pts.append(cur)
            elif kind == "curve":
                _flatten_cubic(cur, op[1], op[2], op[3], flat_tol, pts, 0)
                cur = op[3]
            elif kind == "close":
                pts.append(start)
                cur = start
        return pts

    def arc_length(self):
        """路径总弧长"""
        total = 0.0
        cur = start = None
        for op in self._ops:
            kind = op[0]
            if kind == "move":
                cur = start = op[1]
            elif kind == "line":
                total += _dist(cur, op[1])
                cur = op[1]
            elif kind == "curve":
                total += _cubic_length(cur, op[1], op[2], op[3])
                cur = op[3]
            elif kind == "close":
                total += _dist(cur, start)
                cur = start
        return total

    def split_at_length(self, distance):
        """按弧长切分，返回 (前半, 后半)；切点严格落在曲线上"""
        total = self.arc_length()
        if distance < 0.0 or distance > total + 1e-9:
            raise ValueError("distance out of range")
        distance = min(max(distance, 0.0), total)

        front = PathBuilder()
        back = PathBuilder()
        remaining = distance
        done = False
        cur = start = None

        for op in self._ops:
            kind = op[0]
            if done:
                back._ops.append(op)
                continue
            if kind == "move":
                front._ops.append(op)
                cur = start = op[1]
                continue

            if kind == "close":
                end, seg_len = start, _dist(cur, start)
            elif kind == "line":
                end, seg_len = op[1], _dist(cur, op[1])
            else:  # curve
                end, seg_len = op[3], _cubic_length(cur, op[1], op[2], op[3])

            if remaining > seg_len:
                front._ops.append(op)
                remaining -= seg_len
                cur = end
                if kind == "close":
                    cur = start
                continue

            # 切分点落在本段内
            if kind == "curve":
                lo, hi = 0.0, 1.0
                for _ in range(80):
                    mid_t = 0.5 * (lo + hi)
                    if _cubic_length(cur, op[1], op[2], op[3], 0.0, mid_t) < remaining:
                        lo = mid_t
                    else:
                        hi = mid_t
                t = 0.5 * (lo + hi)
                (q0, q1, q2, q3), (r0, r1, r2, r3), split_pt = _split_cubic(
                    cur, op[1], op[2], op[3], t
                )
                front._ops.append(("curve", q1, q2, q3))
                back._ops.append(("move", split_pt))
                back._ops.append(("curve", r1, r2, r3))
            else:
                seg = end if kind == "line" else start
                t = remaining / seg_len if seg_len > 0 else 0.0
                split_pt = _lerp(cur, seg, t)
                front._ops.append(("line", split_pt))
                back._ops.append(("move", split_pt))
                back._ops.append(("line", seg))
            done = True

        if not done:
            # 切分点在路径末尾（distance == total）
            back._ops.append(("move", cur if cur is not None else (0, 0)))

        front._npoints = sum(
            1 if o[0] in ("move", "line") else 3 if o[0] == "curve" else 0
            for o in front._ops
        )
        back._npoints = sum(
            1 if o[0] in ("move", "line") else 3 if o[0] == "curve" else 0
            for o in back._ops
        )
        return front, back

    # ---- 快照 ----

    def snapshot(self):
        """快照当前状态，返回标记"""
        token = self._next_token
        self._next_token += 1
        self._snapshots[token] = (
            len(self._ops), self._npoints, self._cur, self._start
        )
        return token

    def restore(self, marker):
        """回滚到快照；无效标记抛 ValueError"""
        if marker not in self._snapshots:
            raise ValueError("invalid snapshot marker")
        n, npoints, cur, start = self._snapshots[marker]
        del self._ops[n:]
        self._npoints = npoints
        self._cur = cur
        self._start = start
        return self

    # ---- 查询 ----

    def bounds(self):
        """包围盒 (min_x, min_y, max_x, max_y)；空路径抛 ValueError"""
        xs, ys = [], []
        for op in self._ops:
            for p in op[1:]:
                xs.append(p[0])
                ys.append(p[1])
        if not xs:
            raise ValueError("empty path has no bounds")
        return (min(xs), min(ys), max(xs), max(ys))

    def point_count(self):
        """顶点数"""
        return self._npoints
