from klippy.extras.gcode_arcs import ArcSupport


class DummyGCodeMove:
    def __init__(self):
        self.native_allowed = True
        self.calls = []

    def can_use_native_arc_moves(self):
        return self.native_allowed

    def cmd_G2G3(self, gcmd, target_pos, offset, clockwise, axes):
        self.calls.append((gcmd, target_pos, offset, clockwise, axes))


def make_arc_support(native=True):
    arc_support = ArcSupport.__new__(ArcSupport)
    arc_support.native = native
    arc_support.gcode_move = DummyGCodeMove()
    return arc_support


def test_native_arc_selected_for_xy_non_helical_move():
    arc_support = make_arc_support()

    handled = arc_support._try_native_arc(
        currentPos=(1.0, 0.0, 0.0, 0.0),
        targetPos=[0.0, 1.0, 0.0],
        offset=(-1.0, 0.0),
        clockwise=False,
        gcmd=object(),
        axes=(0, 1, 2),
    )

    assert handled
    assert len(arc_support.gcode_move.calls) == 1


def test_native_arc_disabled_falls_back_to_segments():
    arc_support = make_arc_support(native=False)

    handled = arc_support._try_native_arc(
        currentPos=(1.0, 0.0, 0.0, 0.0),
        targetPos=[0.0, 1.0, 0.0],
        offset=(-1.0, 0.0),
        clockwise=False,
        gcmd=object(),
        axes=(0, 1, 2),
    )

    assert not handled
    assert not arc_support.gcode_move.calls


def test_native_arc_unsupported_plane_falls_back_to_segments():
    arc_support = make_arc_support()

    handled = arc_support._try_native_arc(
        currentPos=(1.0, 0.0, 0.0, 0.0),
        targetPos=[0.0, 1.0, 0.0],
        offset=(-1.0, 0.0),
        clockwise=False,
        gcmd=object(),
        axes=(0, 2, 1),
    )

    assert not handled
    assert not arc_support.gcode_move.calls


def test_native_arc_helical_move_falls_back_to_segments():
    arc_support = make_arc_support()

    handled = arc_support._try_native_arc(
        currentPos=(1.0, 0.0, 0.0, 0.0),
        targetPos=[0.0, 1.0, 1.0],
        offset=(-1.0, 0.0),
        clockwise=False,
        gcmd=object(),
        axes=(0, 1, 2),
    )

    assert not handled
    assert not arc_support.gcode_move.calls


def test_native_arc_active_transform_falls_back_to_segments():
    arc_support = make_arc_support()
    arc_support.gcode_move.native_allowed = False

    handled = arc_support._try_native_arc(
        currentPos=(1.0, 0.0, 0.0, 0.0),
        targetPos=[0.0, 1.0, 0.0],
        offset=(-1.0, 0.0),
        clockwise=False,
        gcmd=object(),
        axes=(0, 1, 2),
    )

    assert not handled
    assert not arc_support.gcode_move.calls
