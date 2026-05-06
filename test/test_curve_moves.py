import math

import pytest

from klippy.extras.arc_geometry import ArcGeometry
from klippy.toolhead import ArcMove, Move


class DummyPrinter:
    def command_error(self, msg):
        return RuntimeError(msg)


class DummyExtruder:
    def calc_junction(self, prev_move, move):
        return move.max_cruise_v2


class DummyToolHead:
    max_accel = 50.0
    max_accel_to_decel = 25.0
    max_velocity = 100.0
    junction_deviation = 0.01
    printer = DummyPrinter()
    extruder = DummyExtruder()


def test_linear_move_exposes_tangent_fields():
    move = Move(
        DummyToolHead(),
        start_pos=(0.0, 0.0, 0.0, 0.0),
        end_pos=(3.0, 4.0, 0.0, 2.0),
        speed=20.0,
    )

    assert move.move_d == pytest.approx(5.0)
    assert move.axes_r == pytest.approx([0.6, 0.8, 0.0, 0.4])
    assert move.start_tangent == pytest.approx((0.6, 0.8, 0.0))
    assert move.end_tangent == pytest.approx((0.6, 0.8, 0.0))


def test_arc_move_exposes_planner_compatibility_fields():
    arc = ArcGeometry(
        start_pos=(1.0, 0.0, 0.0, 0.0),
        end_pos=(0.0, 1.0, 0.0, 2.0),
        offset=(-1.0, 0.0),
        clockwise=False,
    )
    move = ArcMove(DummyToolHead(), arc, speed=5.0)

    assert move.start_pos == pytest.approx(arc.start_pos)
    assert move.end_pos == pytest.approx(arc.end_pos)
    assert move.move_d == pytest.approx(0.5 * math.pi)
    assert move.axes_d == pytest.approx([-1.0, 1.0, 0.0, 2.0])
    assert move.axes_r[:3] == pytest.approx([0.0, 1.0, 0.0])
    assert move.axes_r[3] == pytest.approx(2.0 / (0.5 * math.pi))
    assert move.start_tangent == pytest.approx((0.0, 1.0, 0.0))
    assert move.end_tangent == pytest.approx((-1.0, 0.0, 0.0), abs=1.0e-12)
    assert move.curvature == pytest.approx(1.0)
    assert move.max_cruise_v2 == pytest.approx(5.0**2)
    assert move.delta_v2 == pytest.approx(2.0 * move.move_d * DummyToolHead.max_accel)
    assert move.smooth_delta_v2 == pytest.approx(
        2.0 * move.move_d * DummyToolHead.max_accel_to_decel
    )


def test_arc_move_evaluates_underlying_geometry():
    arc = ArcGeometry(
        start_pos=(1.0, 0.0, 0.0, 0.0),
        end_pos=(0.0, 1.0, 0.0, 0.0),
        offset=(-1.0, 0.0),
        clockwise=False,
    )
    move = ArcMove(DummyToolHead(), arc, speed=20.0)
    mid = math.sqrt(0.5)

    assert move.position_at(move.move_d * 0.5) == pytest.approx(
        (mid, mid, 0.0, 0.0)
    )
    assert move.tangent_at(0.0) == pytest.approx((0.0, 1.0, 0.0, 0.0))
    assert move.normal_at(0.0) == pytest.approx((-1.0, 0.0, 0.0, 0.0))


def test_linear_junction_still_uses_equivalent_tangents():
    prev_move = Move(
        DummyToolHead(),
        start_pos=(0.0, 0.0, 0.0, 0.0),
        end_pos=(10.0, 0.0, 0.0, 0.0),
        speed=20.0,
    )
    move = Move(
        DummyToolHead(),
        start_pos=(10.0, 0.0, 0.0, 0.0),
        end_pos=(10.0, 10.0, 0.0, 0.0),
        speed=20.0,
    )

    move.calc_junction(prev_move)

    assert move.max_start_v2 == pytest.approx(
        DummyToolHead.junction_deviation
        * DummyToolHead.max_accel
        * math.sin(math.pi / 4.0)
        / (1.0 - math.sin(math.pi / 4.0))
    )


def test_line_to_tangent_arc_has_no_scv_slowdown():
    prev_move = Move(
        DummyToolHead(),
        start_pos=(1.0, -10.0, 0.0, 0.0),
        end_pos=(1.0, 0.0, 0.0, 0.0),
        speed=20.0,
    )
    arc = ArcGeometry(
        start_pos=(1.0, 0.0, 0.0, 0.0),
        end_pos=(0.0, 1.0, 0.0, 0.0),
        offset=(-1.0, 0.0),
        clockwise=False,
    )
    move = ArcMove(DummyToolHead(), arc, speed=20.0)

    move.calc_junction(prev_move)

    assert move.max_start_v2 == pytest.approx(move.max_cruise_v2)


def test_arc_curvature_caps_cruise_speed():
    arc = ArcGeometry(
        start_pos=(1.0, 0.0, 0.0, 0.0),
        end_pos=(0.0, 1.0, 0.0, 0.0),
        offset=(-1.0, 0.0),
        clockwise=False,
    )
    move = ArcMove(DummyToolHead(), arc, speed=100.0)

    assert move.max_cruise_v2 == pytest.approx(DummyToolHead.max_accel)
    assert move.min_move_t == pytest.approx(move.move_d / math.sqrt(50.0))
