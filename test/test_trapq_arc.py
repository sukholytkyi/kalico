import math

import pytest

from klippy import chelper


def extract_moves(trapq, count=8):
    ffi_main, ffi_lib = chelper.get_ffi()
    moves = ffi_main.new("struct pull_move[]", count)
    found = ffi_lib.trapq_extract_old(trapq, moves, count, 0.0, 999.0)
    return [
        (
            moves[i].print_time,
            moves[i].move_t,
            moves[i].start_v,
            moves[i].accel,
            moves[i].start_x,
            moves[i].start_y,
            moves[i].start_z,
            moves[i].x_r,
            moves[i].y_r,
            moves[i].z_r,
        )
        for i in range(found)
    ]


def test_trapq_append_arc_exports_arc_phase_start():
    ffi_main, ffi_lib = chelper.get_ffi()
    trapq = ffi_main.gc(ffi_lib.trapq_alloc(), ffi_lib.trapq_free)
    path_length = 0.5 * math.pi

    ffi_lib.trapq_append_arc(
        trapq,
        0.0,
        0.0,
        path_length,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.5 * math.pi,
        path_length,
        0.0,
        1.0,
        1.0,
        0.0,
    )
    ffi_lib.trapq_finalize_moves(trapq, 999.0, 0.0)

    moves = extract_moves(trapq)

    assert len(moves) == 1
    assert moves[0][4] == pytest.approx(1.0)
    assert moves[0][5] == pytest.approx(0.0)
    assert moves[0][6] == pytest.approx(0.0)
    assert moves[0][7] == pytest.approx(0.0)
    assert moves[0][8] == pytest.approx(1.0)
    assert moves[0][9] == pytest.approx(0.0)


def test_trapq_append_arc_splits_phase_start_positions():
    ffi_main, ffi_lib = chelper.get_ffi()
    trapq = ffi_main.gc(ffi_lib.trapq_alloc(), ffi_lib.trapq_free)
    path_length = 0.5 * math.pi
    accel_t = 0.5
    accel = 4.0
    accel_d = 0.5 * accel * accel_t * accel_t
    cruise_v = accel * accel_t
    cruise_t = (path_length - accel_d) / cruise_v

    ffi_lib.trapq_append_arc(
        trapq,
        0.0,
        accel_t,
        cruise_t,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.5 * math.pi,
        path_length,
        0.0,
        0.0,
        cruise_v,
        accel,
    )
    ffi_lib.trapq_finalize_moves(trapq, 999.0, 0.0)

    moves = extract_moves(trapq)
    theta = accel_d / path_length * (0.5 * math.pi)

    assert len(moves) == 2
    assert moves[0][4] == pytest.approx(math.cos(theta))
    assert moves[0][5] == pytest.approx(math.sin(theta))
    assert moves[0][7] == pytest.approx(-math.sin(theta))
    assert moves[0][8] == pytest.approx(math.cos(theta))
    assert moves[1][4] == pytest.approx(1.0)
    assert moves[1][5] == pytest.approx(0.0)


def test_itersolve_check_active_uses_arc_axis_mask():
    ffi_main, ffi_lib = chelper.get_ffi()
    trapq = ffi_main.gc(ffi_lib.trapq_alloc(), ffi_lib.trapq_free)
    path_length = 0.5 * math.pi

    ffi_lib.trapq_append_arc(
        trapq,
        2.0,
        0.0,
        path_length,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.5 * math.pi,
        path_length,
        0.0,
        1.0,
        1.0,
        0.0,
    )
    sk = ffi_main.gc(ffi_lib.cartesian_stepper_alloc(b"x"), ffi_lib.free)
    ffi_lib.itersolve_set_trapq(sk, trapq)

    assert ffi_lib.itersolve_check_active(sk, 10.0) == pytest.approx(2.0)


def test_itersolve_check_active_still_skips_inactive_line_axis():
    ffi_main, ffi_lib = chelper.get_ffi()
    trapq = ffi_main.gc(ffi_lib.trapq_alloc(), ffi_lib.trapq_free)

    ffi_lib.trapq_append(
        trapq,
        2.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        1.0,
        1.0,
        0.0,
    )
    sk = ffi_main.gc(ffi_lib.cartesian_stepper_alloc(b"x"), ffi_lib.free)
    ffi_lib.itersolve_set_trapq(sk, trapq)

    assert ffi_lib.itersolve_check_active(sk, 10.0) == pytest.approx(0.0)
