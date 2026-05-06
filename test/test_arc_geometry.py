import math

import pytest

from klippy.extras.arc_geometry import ArcGeometry, ArcGeometryError


def assert_close_tuple(actual, expected, abs_tol=1.0e-9):
    assert len(actual) == len(expected)
    for a, e in zip(actual, expected):
        assert a == pytest.approx(e, abs=abs_tol)


def test_quarter_ccw_arc_geometry():
    arc = ArcGeometry(
        start_pos=(1.0, 0.0, 0.0, 0.0),
        end_pos=(0.0, 1.0, 0.0, 0.0),
        offset=(-1.0, 0.0),
        clockwise=False,
    )
    assert arc.center == pytest.approx((0.0, 0.0))
    assert arc.radius == pytest.approx(1.0)
    assert arc.angular_travel == pytest.approx(0.5 * math.pi)
    assert arc.path_length == pytest.approx(0.5 * math.pi)
    assert arc.curvature == pytest.approx(1.0)
    assert_close_tuple(arc.start_tangent, (0.0, 1.0, 0.0, 0.0))
    assert_close_tuple(arc.end_tangent, (-1.0, 0.0, 0.0, 0.0))
    mid = math.sqrt(0.5)
    assert_close_tuple(arc.position_at(arc.path_length * 0.5), (mid, mid, 0.0, 0.0))
    assert_close_tuple(arc.position_at(arc.path_length), arc.end_pos)


def test_quarter_cw_arc_geometry():
    arc = ArcGeometry(
        start_pos=(0.0, 1.0, 0.0, 0.0),
        end_pos=(1.0, 0.0, 0.0, 0.0),
        offset=(0.0, -1.0),
        clockwise=True,
    )
    assert arc.angular_travel == pytest.approx(-0.5 * math.pi)
    assert arc.path_length == pytest.approx(0.5 * math.pi)
    assert_close_tuple(arc.start_tangent, (1.0, 0.0, 0.0, 0.0))
    assert_close_tuple(arc.end_tangent, (0.0, -1.0, 0.0, 0.0))


def test_full_circle_ccw_arc_geometry():
    arc = ArcGeometry(
        start_pos=(1.0, 0.0, 0.0, 0.0),
        end_pos=(1.0, 0.0, 0.0, 0.0),
        offset=(-1.0, 0.0),
        clockwise=False,
    )
    assert arc.angular_travel == pytest.approx(2.0 * math.pi)
    assert arc.path_length == pytest.approx(2.0 * math.pi)
    assert_close_tuple(arc.position_at(arc.path_length), arc.end_pos)


def test_extrusion_ratio_uses_true_arc_length():
    arc = ArcGeometry(
        start_pos=(2.0, 0.0, 0.0, 10.0),
        end_pos=(-2.0, 0.0, 0.0, 14.0),
        offset=(-2.0, 0.0),
        clockwise=False,
    )
    assert arc.radius == pytest.approx(2.0)
    assert arc.angular_travel == pytest.approx(math.pi)
    assert arc.path_length == pytest.approx(2.0 * math.pi)
    assert arc.e_delta == pytest.approx(4.0)
    assert arc.e_ratio == pytest.approx(4.0 / (2.0 * math.pi))
    assert arc.position_at(arc.path_length * 0.25)[3] == pytest.approx(11.0)


def test_rejects_zero_offset():
    with pytest.raises(ArcGeometryError, match="non-zero center offset"):
        ArcGeometry((0.0, 0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0), (0.0, 0.0), False)


def test_rejects_radius_mismatch():
    with pytest.raises(ArcGeometryError, match="radius mismatch"):
        ArcGeometry((1.0, 0.0, 0.0, 0.0), (0.0, 2.0, 0.0, 0.0), (-1.0, 0.0), False)


def test_rejects_mvp_helical_arc():
    with pytest.raises(ArcGeometryError, match="helical"):
        ArcGeometry((1.0, 0.0, 0.0, 0.0), (0.0, 1.0, 1.0, 0.0), (-1.0, 0.0), False)


def test_can_compute_helical_arc_when_enabled():
    arc = ArcGeometry(
        start_pos=(1.0, 0.0, 0.0, 0.0),
        end_pos=(0.0, 1.0, 1.0, 0.0),
        offset=(-1.0, 0.0),
        clockwise=False,
        allow_helical=True,
    )
    h = 1.0 / (0.5 * math.pi)
    assert arc.path_length == pytest.approx(math.hypot(0.5 * math.pi, 1.0))
    assert arc.curvature == pytest.approx(1.0 / (1.0 + h * h))
    tangent = arc.tangent_at(0.0)
    assert math.sqrt(sum(v * v for v in tangent[:3])) == pytest.approx(1.0)
