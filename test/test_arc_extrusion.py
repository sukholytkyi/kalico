import math

import pytest

from klippy.extras.arc_geometry import ArcGeometry
from klippy.kinematics.extruder import PrinterExtruder
from klippy.toolhead import ArcMove


class DummyPrinter:
    def command_error(self, msg):
        return RuntimeError(msg)


class DummyHeater:
    can_extrude = True


class DummyStepper:
    pressure_advance = 0.05
    per_move_pressure_advance = False


class DummyExtruder(PrinterExtruder):
    printer = DummyPrinter()
    heater = DummyHeater()
    nozzle_diameter = 0.4
    filament_area = 1.0
    max_extrude_ratio = 10.0
    max_e_dist = 1.0
    max_e_velocity = 25.0
    max_e_accel = 500.0
    instant_corner_v = 1.0
    extruder_stepper = DummyStepper()

    def __init__(self):
        self.trapq_calls = []
        self.last_position = 0.0
        self.trapq = object()

    def trapq_append(self, *args):
        self.trapq_calls.append(args)


class DummyToolHead:
    max_accel = 10000.0
    max_accel_to_decel = 5000.0
    max_velocity = 200.0
    junction_deviation = 0.01
    printer = DummyPrinter()

    def __init__(self):
        self.extruder = DummyExtruder()


def make_full_circle_arc_move(e_delta=1.0, speed=100.0):
    arc = ArcGeometry(
        start_pos=(1.0, 0.0, 0.0, 0.0),
        end_pos=(1.0, 0.0, 0.0, e_delta),
        offset=(-1.0, 0.0),
        clockwise=False,
    )
    return ArcMove(DummyToolHead(), arc, speed=speed)


def test_arc_move_marks_full_circle_as_xy_motion():
    move = make_full_circle_arc_move()

    assert move.axes_d[:2] == pytest.approx([0.0, 0.0])
    assert move.has_xy_motion
    assert move.move_d == pytest.approx(2.0 * math.pi)
    assert move.axes_r[3] == pytest.approx(1.0 / (2.0 * math.pi))


def test_full_circle_arc_is_not_limited_as_extrude_only_move():
    extruder = DummyExtruder()
    move = make_full_circle_arc_move(e_delta=2.0)

    PrinterExtruder.check_move(extruder, move)


def test_full_circle_arc_uses_pressure_advance_when_extruding():
    extruder = DummyExtruder()
    move = make_full_circle_arc_move(e_delta=1.0)
    move.accel_t = 0.1
    move.cruise_t = 0.2
    move.decel_t = 0.1
    move.start_v = 20.0
    move.cruise_v = 50.0
    move.accel = 1000.0

    PrinterExtruder.move(extruder, 12.0, move)

    assert extruder.last_position == pytest.approx(1.0)
    call = extruder.trapq_calls[0]
    assert call[0] is extruder.trapq
    assert call[1] == pytest.approx(12.0)
    assert call[8] == pytest.approx(1.0)
    assert call[9] == pytest.approx(extruder.extruder_stepper.pressure_advance)
    assert call[10] == pytest.approx(0.0)
    assert call[11] == pytest.approx(move.start_v * move.axes_r[3])
    assert call[12] == pytest.approx(move.cruise_v * move.axes_r[3])
    assert call[13] == pytest.approx(move.accel * move.axes_r[3])
