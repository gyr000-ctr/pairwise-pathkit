import math

import pytest

import pathkit

def _raises(fn, exc=ValueError):
    try:
        fn()
    except exc:
        return True
    return False


def _pt_seg_dist(p, a, b):
    ax, ay = a
    bx, by = b
    px, py = p
    dx, dy = bx - ax, by - ay
    if dx == 0 and dy == 0:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))

def _curve():
    b = pathkit.PathBuilder()
    b.move_to(0, 0)
    b.curve_to((0, 1), (1, 1), (1, 0))
    return b


def _flat_error(tolerance=0.02):
    b = _curve()
    pts = b.flatten(tolerance)
    worst = 0.0
    for i in range(801):
        t = i / 800.0
        mt = 1 - t
        x = 3 * mt * mt * t * 0 + 3 * mt * t * t * 1 + t ** 3 * 1
        y = 3 * mt * mt * t * 1 + 3 * mt * t * t * 1 + t ** 3 * 0
        d = min(_pt_seg_dist((x, y), pts[j], pts[j + 1]) for j in range(len(pts) - 1))
        worst = max(worst, d)
    return worst <= tolerance + 1e-9


def _arc_accuracy():
    b = _curve()
    pts = b.flatten(0.0005)
    dense = sum(math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1]) for i in range(len(pts) - 1))
    return abs(b.arc_length() - dense) <= max(0.005 * dense, 1e-6)


def _split_point_on_curve():
    b = _curve()
    a, c = b.split_at_length(0.5)
    pa = a.flatten(0.01)[-1]
    pc = c.flatten(0.01)[0]
    if math.hypot(pa[0] - pc[0], pa[1] - pc[1]) > 1e-6:
        return False
    total = b.arc_length()
    return abs(a.arc_length() + c.arc_length() - total) <= 0.01 * total


def _line_arc():
    b = pathkit.PathBuilder()
    b.move_to(0, 0)
    b.line_to(3, 4)
    return abs(b.arc_length() - 5.0) < 1e-9


def _replay():
    b = _curve()
    before = b.flatten(0.02)
    m = b.snapshot()
    b.line_to(9, 9)
    b.restore(m)
    return b.flatten(0.02) == before


def _limit_rollback():
    b = pathkit.PathBuilder()
    b.move_to(0, 0)
    try:
        for i in range(20000):
            b.line_to(i, i)
    except ValueError:
        return b.point_count() <= 10000
    return False


def _closed():
    b = _curve()
    b.close()
    pts = b.flatten(0.01)
    return pts[0] == pts[-1]

CASES = [
        ("flatten_endpoints", lambda: _curve().flatten(0.02)[0] == (0, 0) and _curve().flatten(0.02)[-1] == (1, 0)),
        ("flatten_error", lambda: _flat_error()),
        ("flatten_error_tight", lambda: _flat_error(0.005)),
        ("arc_accuracy", lambda: _arc_accuracy()),
        ("arc_line", lambda: _line_arc()),
        ("split_on_curve", lambda: _split_point_on_curve()),
        ("split_zero", lambda: _raises(lambda: _curve().split_at_length(-1))),
        ("replay", lambda: _replay()),
        ("restore_bad", lambda: _raises(lambda: _curve().restore(9999))),
        ("limit", lambda: _limit_rollback()),
        ("tolerance", lambda: _raises(lambda: _curve().flatten(0.0))),
        ("empty_bounds", lambda: _raises(lambda: pathkit.PathBuilder().bounds())),
        ("closed", lambda: _closed()),
        ("curve_points", lambda: _raises(lambda: (lambda b: (b.move_to(0, 0), b.curve_to((0, 1), (1, 0))))(pathkit.PathBuilder()))),
        ("count", lambda: len(_curve().flatten(0.02)) >= 2),
    ]


@pytest.mark.parametrize("name,check", CASES, ids=[c[0] for c in CASES])
def test_spec(name, check):
    assert check() is True
