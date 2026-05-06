from klippy.extras.gcode_move import GCodeMove


class DummyGCode:
    class Coord(tuple):
        def __new__(cls, *values):
            return tuple.__new__(cls, values)

    def register_command(self, *args, **kwargs):
        pass


class DummyToolHead:
    def __init__(self):
        self.arc_moves = []

    def move(self, newpos, speed):
        pass

    def move_arc(self, arc, speed):
        self.arc_moves.append((arc, speed))

    def get_position(self):
        return [0.0, 0.0, 0.0, 0.0]


class DummyTransform:
    def move(self, newpos, speed):
        pass

    def get_position(self):
        return [0.0, 0.0, 0.0, 0.0]


class DummyPrinter:
    def __init__(self):
        self.gcode = DummyGCode()
        self.toolhead = DummyToolHead()

    def register_event_handler(self, *args, **kwargs):
        pass

    def lookup_object(self, name, default=None):
        if name == "gcode":
            return self.gcode
        if name == "toolhead":
            return self.toolhead
        return default

    def config_error(self, msg):
        return RuntimeError(msg)


class DummyConfig:
    def __init__(self):
        self.printer = DummyPrinter()

    def get_printer(self):
        return self.printer


def test_native_arc_moves_allowed_without_transform():
    gcode_move = GCodeMove(DummyConfig())
    gcode_move._handle_ready()

    assert not gcode_move.has_move_transform()
    assert gcode_move.can_use_native_arc_moves()


def test_native_arc_moves_blocked_by_active_transform():
    gcode_move = GCodeMove(DummyConfig())
    gcode_move._handle_ready()

    gcode_move.set_move_transform(DummyTransform())

    assert gcode_move.has_move_transform()
    assert not gcode_move.can_use_native_arc_moves()


def test_toolhead_restore_is_not_treated_as_active_transform():
    gcode_move = GCodeMove(DummyConfig())
    gcode_move._handle_ready()
    toolhead = gcode_move.toolhead

    gcode_move.set_move_transform(DummyTransform())
    gcode_move.set_move_transform(toolhead, force=True)

    assert not gcode_move.has_move_transform()
    assert gcode_move.can_use_native_arc_moves()


class DummyGCmd:
    def __init__(self, params):
        self.params = params

    def get_command_parameters(self):
        return dict(self.params)

    def get_commandline(self):
        return "G2"

    def error(self, msg):
        return RuntimeError(msg)


def test_gcode_move_native_arc_updates_state_and_calls_toolhead():
    config = DummyConfig()
    gcode_move = GCodeMove(config)
    gcode_move._handle_ready()
    gcode_move.last_position = [1.0, 0.0, 0.0, 10.0]

    gcode_move.cmd_G2G3(
        DummyGCmd({"E": "12", "F": "600"}),
        target_pos=[0.0, 1.0, 0.0],
        offset=(-1.0, 0.0),
        clockwise=False,
        axes=(0, 1, 2),
    )

    assert gcode_move.last_position == [0.0, 1.0, 0.0, 12.0]
    assert gcode_move.speed == 10.0
    arc, speed = config.printer.toolhead.arc_moves[0]
    assert arc.start_pos == (1.0, 0.0, 0.0, 10.0)
    assert arc.end_pos == (0.0, 1.0, 0.0, 12.0)
    assert speed == 10.0
