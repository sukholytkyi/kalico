# Native G2/G3 Arc Motion Research

This note summarizes the current Kalico arc implementation and a possible path
to native `G2`/`G3` support. It is intended as future design context for native
curve work, especially before attempting native `G5`/`G5.1` splines.

## Current Behavior

Kalico currently supports `G2`/`G3` through `klippy/extras/gcode_arcs.py`.
That module parses arc commands and converts each arc into many `G1` line
moves. The rest of the motion stack only sees normal linear moves.

Current arc path:

```text
G2/G3
 -> klippy/extras/gcode_arcs.py
 -> generated G1 commands
 -> klippy/extras/gcode_move.py
 -> ToolHead.move()
 -> Move
 -> LookAheadQueue / square_corner_velocity
 -> trapq_append()
 -> itersolve_generate_steps()
 -> stepcompress
 -> MCU queue_step commands
 -> src/stepper.c step pulses
```

This means arcs are not native motion primitives today. They are polygonized
before lookahead and step generation.

Target native arc path:

```text
G2/G3
 -> parse arc geometry
 -> native ArcMove
 -> tangent-aware LookAheadQueue / square_corner_velocity
 -> curvature-aware speed limiting
 -> curve-aware trapq_append_arc()
 -> itersolve_generate_steps()
 -> stepcompress
 -> MCU queue_step commands
 -> src/stepper.c step pulses
```

The key difference is that the arc remains a curve until the normal step
generation process asks for positions at specific times.

## Why Native Arcs

Segmented arcs create artificial chord-to-chord junctions. Kalico's
lookahead/SCV code treats each of those junctions like a small corner. With
fine enough segmentation the effect is reduced, but the planner still sees a
polygon instead of a continuous curve.

Native arcs could:

- Remove artificial segment junctions inside an arc.
- Preserve true tangent continuity within the arc.
- Reduce the number of host-side moves.
- Avoid chordal path error from `G1` segmentation.
- Provide a smaller proving ground for native curve support before native
  `G5`/`G5.1`.

## Existing Motion Model

Kalico's core motion primitive is currently a straight-line move with
trapezoidal velocity.

The `Move` class in `klippy/toolhead.py` stores:

- Start position.
- End position.
- Axis deltas.
- Unit direction vector, `axes_r`.
- Move distance.
- Start/cruise/end velocities.
- Acceleration timing.

The C trap queue stores linear move data in `klippy/chelper/trapq.h`:

```c
struct move {
    double print_time, move_t;
    double start_v, half_accel;
    struct coord start_pos, axes_r;
    struct list_node node;
};
```

`move_get_coord()` in `klippy/chelper/trapq.c` evaluates position as:

```text
position = start_pos + axes_r * distance
```

Native arcs require this to become move-type aware.

## Required Change Areas

### Code Touch Point Map

This section maps the concrete functions and contracts that native arcs need to
cross.

#### G-code State And Move Dispatch

`klippy/extras/gcode_move.py`

- `GCodeMove.cmd_G1()` parses `XYZEF`, updates `last_position`, handles
  absolute and relative coordinate state, applies extrusion and speed factors,
  and calls `self.move_with_transform(self.last_position, self.speed)`.
- `GCodeMove.set_move_transform()` allows modules such as bed mesh or skew
  correction to intercept moves by replacing `move_with_transform`.
- Native arcs need an equivalent dispatch path that preserves this state machine.
  A native arc should not bypass transforms unless those transforms are
  intentionally declared line-only.

Implication: a native `G2`/`G3` entry point should probably live near
`gcode_move.py` rather than only in `gcode_arcs.py`, because `gcode_move.py`
owns the canonical G-code position, extrusion factor, speed factor, and transform
chain.

#### Toolhead Move Creation

`klippy/toolhead.py`

- `ToolHead.move()` creates a `Move`, checks kinematic and extruder limits,
  updates `commanded_pos`, and submits the move to `LookAheadQueue`.
- A native arc needs a sibling entry point, for example `ToolHead.move_arc()`,
  or a generalized `ToolHead.move()` path that can instantiate line or arc move
  objects.
- `Move.__init__()` currently computes `axes_d`, `move_d`, `axes_r`,
  `max_cruise_v2`, `delta_v2`, and `smooth_delta_v2` assuming a straight line.
  `ArcMove` must provide compatible fields or the planner must be generalized.

Important compatibility fields for the existing planner:

```text
move.move_d
move.axes_d
move.axes_r
move.is_kinematic_move
move.max_cruise_v2
move.delta_v2
move.smooth_delta_v2
move.min_move_t
move.set_junction()
move.timing_callbacks
```

For native arcs, `axes_r[:3]` is no longer a constant XYZ direction through the
whole move. It can remain as a compatibility value only if all logic that needs
real path direction uses start/end tangents instead.

#### Lookahead

`klippy/toolhead.py`

- `LookAheadQueue.add_move()` calls `move.calc_junction(previous_move)`.
- `LookAheadQueue.flush()` uses `max_start_v2`, `max_smoothed_v2`, `delta_v2`,
  and `smooth_delta_v2` to assign start/cruise/end speeds.
- `Move.calc_junction()` currently compares `self.axes_r[:3]` and
  `prev_move.axes_r[:3]`.

Required native-arc change:

```text
old: compare previous axes_r vs current axes_r
new: compare previous end_tangent vs current start_tangent
```

The rest of lookahead can stay closer to the current shape if `ArcMove` exposes
the same scalar velocity-planning fields.

#### Trap Queue And Python-to-C Boundary

`klippy/toolhead.py`

- `ToolHead.__init__()` stores `ffi_lib.trapq_append` as `self.trapq_append`.
- `ToolHead._process_moves()` sends each kinematic move to `trapq_append()`
  using timing, start position, linear `axes_r`, start velocity, cruise velocity,
  and acceleration.

`klippy/chelper/trapq.h`

- `struct move` currently stores only `start_pos` and linear `axes_r`.
- `trapq_append()` is the only public append function for XYZ moves.
- `move_get_coord()` is the shared position evaluator used by kinematics.

Required native-arc change:

- Add move type metadata to `struct move`, or add a separate curve structure.
- Add a C append function, for example `trapq_append_arc()`.
- Update Python FFI declarations in `klippy/chelper/__init__.py`.
- Make `move_get_coord()` evaluate either a line or an arc.
- Keep `trapq_append()` intact for existing linear moves and extruder trapq use.

#### Kinematic Step Generation

`klippy/chelper/itersolve.c`

- `itersolve_generate_steps()` iterates over trapq moves.
- Stepper callbacks call into `move_get_coord()` for XYZ position.
- `check_active()` currently tests `m->axes_r.x/y/z != 0.0` to skip inactive
  moves.

Kinematic callbacks already depend on `move_get_coord()`:

- Cartesian returns one coordinate component.
- CoreXY returns `x + y` or `x - y`.
- Delta, polar, winch, rotary delta, and others also evaluate coordinates through
  `move_get_coord()`.

Required native-arc change:

- If `move_get_coord()` becomes curve-aware, most kinematic callbacks can remain
  unchanged.
- `itersolve.c::check_active()` must not depend only on constant `axes_r`, because
  an arc can have nonzero motion on an axis even when the compatibility direction
  field is misleading.

#### Input Shaper

`klippy/chelper/kin_shaper.c`

- The shaper code samples positions across neighboring moves.
- It uses `move_get_coord()` in some paths, but also reads `m->axes_r.axis[]`
  directly for axis position helpers.

Required native-arc change:

- Audit and update direct `axes_r` assumptions.
- Prefer `move_get_coord()` for sampled positions so shaper behavior follows the
  native curve.

#### Extruder And Pressure Advance

`klippy/kinematics/extruder.py`

- `PrinterExtruder.check_move()` uses `move.axes_r[3]` as E-per-path-distance.
- `PrinterExtruder.calc_junction()` limits E ratio changes between moves using
  `move.axes_r[3] - prev_move.axes_r[3]`.
- `PrinterExtruder.move()` queues extruder motion into its own linear trapq.
  It derives E velocity and acceleration from `move.start_v`, `move.cruise_v`,
  `move.accel`, and `move.axes_r[3]`.

`klippy/chelper/kin_extruder.c`

- Extruder step generation uses the extruder trapq, not the XYZ trapq.
- Pressure advance is calculated from nominal extruder position and velocity.
- PA metadata is currently packed into extruder trapq `axes_r.y` and `axes_r.z`.

Implication: native XYZ arcs do not require the extruder trapq itself to become
curved. The extruder path is one-dimensional E over time. However, `ArcMove`
must expose a correct and stable `axes_r[3]` equal to:

```text
extrusion_ratio = E_delta / true_arc_path_distance
```

That lets existing extruder checks, E junction limits, and PA math continue to
operate on the native arc's true path length.

### `klippy/extras/gcode_arcs.py`

This module should continue to own `G2`/`G3` and `G17`/`G18`/`G19` parsing, but
it should not generate `G1` commands when native mode is enabled.

For a first native mode, it should parse and validate:

- `G2` clockwise arcs.
- `G3` counter-clockwise arcs.
- `G17` XY plane.
- `I`/`J` center offsets.
- Optional `E` extrusion.
- Optional `F` feedrate.

The existing segmented implementation should remain available as a fallback.
This is useful for compatibility, debugging, and emergency disablement.

### `klippy/extras/gcode_move.py`

`gcode_move.py` should preserve normal G-code state while forwarding native arc
moves:

- Absolute and relative coordinate state.
- Absolute and relative extrusion state.
- Feedrate state.
- Current G-code position.
- Final position update after the arc.
- Existing move transform behavior.

The final G-code position should be the arc target, exactly as with a completed
linear `G1` move.

### `klippy/toolhead.py`

The planner needs a native arc move object. This can be either:

- A separate `ArcMove` beside the existing `Move`, or
- A generalized `Move` with `move_type = line|arc`.

Both line and arc moves should expose the same planner-facing interface:

- `move_d`
- `start_pos`
- `end_pos`
- `start_tangent`
- `end_tangent`
- `max_cruise_v2`
- `delta_v2`
- `smooth_delta_v2`
- Position evaluation by distance

Linear moves can map `start_tangent` and `end_tangent` to their existing
`axes_r`. Arc moves derive tangents from arc geometry.

### Lookahead And SCV

`Move.calc_junction()` currently compares linear direction vectors. Native arcs
need junction calculation based on real boundary tangents:

- Line to line: previous line direction vs current line direction.
- Line to arc: previous line direction vs arc start tangent.
- Arc to line: arc end tangent vs current line direction.
- Arc to arc: previous arc end tangent vs current arc start tangent.

There should be no internal SCV junctions inside one native arc. Internal arc
speed is controlled by curvature, not by artificial chord corners.

### `klippy/chelper/trapq.c` And `trapq.h`

The C trap queue must be extended so a queued move can be either linear or arc
motion. The existing `trapq_append()` should remain for linear moves. Native arcs
need a new append path, for example `trapq_append_arc()`, that stores enough arc
metadata for `move_get_coord()`.

`move_get_coord()` should dispatch by move type:

```text
line:
  position = start_pos + axes_r * distance

arc:
  theta = start_theta + signed_distance / radius
  x = center_x + radius * cos(theta)
  y = center_y + radius * sin(theta)
  z = start_z + z_r * distance
```

For helical arcs, the non-planar axis is linear over the same normalized path
distance.

### `itersolve` And Step Compression

`itersolve_generate_steps()` may not need to know the move type if
`move_get_coord()` returns correct curve coordinates. However, native arcs must be
tested because step times on a curve are less regular than step times on a line.

Native arcs should reduce host move count, but they may reduce step compression
efficiency. This needs measurement instead of assumption.

## Native Arc Geometry

An internal native arc descriptor would need at least:

- Plane: `G17` XY, `G18` XZ, or `G19` YZ.
- Center position.
- Radius.
- Start angle.
- Angular travel.
- Clockwise/counter-clockwise direction.
- Linear axis delta for helical arcs.
- Total path length.
- Start tangent vector.
- End tangent vector.

For a planar arc:

```text
theta = start_theta + signed_distance / radius
x = center_x + radius * cos(theta)
y = center_y + radius * sin(theta)
```

For helical arcs, the non-planar axis advances linearly over the same traveled
distance fraction.

Minimum MVP descriptor:

```text
plane = XY
center = (cx, cy)
radius = r
start_angle = atan2(start_y - cy, start_x - cx)
angular_travel = signed angle from start to target
direction = clockwise | counter-clockwise
target = final XYZ/E position
move_d = abs(radius * angular_travel)
start_tangent = tangent at start_angle
end_tangent = tangent at start_angle + angular_travel
```

For a later helical implementation:

```text
planar_d = abs(radius * angular_travel)
linear_d = target[helical_axis] - start[helical_axis]
move_d = hypot(planar_d, linear_d)
```

## SCV And Cornering

Current SCV applies at boundaries between `Move` objects. For segmented arcs,
every generated line segment creates another junction.

Native arcs need two separate concepts:

1. Junction speed between real moves.
2. Curvature speed inside the arc.

For native arc junctions, lookahead should use tangents:

- Line to arc: compare line direction to arc start tangent.
- Arc to line: compare arc end tangent to line direction.
- Arc to arc: compare previous arc end tangent to next arc start tangent.

For motion inside the arc, the toolhead has centripetal acceleration:

```text
a_normal = v^2 / radius
```

So native arcs need a curvature-based speed cap:

```text
v <= sqrt(max_accel * radius)
```

A more complete model should account for combined tangential and normal
acceleration:

```text
sqrt(a_tangential^2 + a_normal^2) <= max_accel
```

Ignoring this would allow small-radius arcs to exceed configured acceleration
limits even if the tangential trapezoid acceleration looks valid.

For the MVP, a conservative internal arc speed cap is acceptable:

```text
arc_max_v = sqrt(max_accel * radius)
```

That cap should be applied before final lookahead velocity planning. A later
version can reduce available tangential acceleration while moving through a curve
so the combined acceleration vector remains inside the configured limit.

## Pressure Advance And SCV With Native Arcs

Native `G2`/`G3` changes SCV behavior more directly than pressure advance
behavior.

With the current segmented implementation, one arc becomes many small linear
moves:

```text
arc -> G1 segment -> G1 segment -> G1 segment -> ...
```

SCV sees each chord boundary as a junction. These are artificial corners created
by polygonization, not real features in the requested toolpath. Fine
segmentation reduces the angle at each fake junction, but it does not remove the
fake junctions.

With native arcs, one arc remains one curve:

```text
arc -> native ArcMove
```

SCV should only apply at real move boundaries:

```text
line -> arc
arc -> line
arc -> arc
```

Those junctions should be calculated from tangents:

```text
line -> arc: line direction vs arc start tangent
arc -> line: arc end tangent vs line direction
arc -> arc: previous arc end tangent vs next arc start tangent
```

Inside a native arc, SCV should not be the limiting mechanism. Internal arc speed
should be limited by curvature:

```text
a_normal = v^2 / radius
v <= sqrt(max_accel * radius)
```

Pressure advance is different. It compensates extrusion pressure lag based on
changes in extrusion flow. With segmented arcs, pressure advance sees many small
linear moves. With native arcs, it should see continuous extrusion over the
native arc distance.

For a native arc with extrusion proportional to path distance:

```text
E(s) = E_start + e_ratio * s
dE/dt = dE/ds * ds/dt
```

Here `s` is distance along the arc. Pressure advance should operate from the
native move's path velocity profile: acceleration, cruise, deceleration, and real
junction speed changes. It should not react to artificial internal chord
boundaries because those boundaries no longer exist.

Practical expected effects:

- SCV should improve because fake internal arc junctions disappear.
- Velocity through curved paths should become smoother.
- Line-to-arc and arc-to-line transitions should depend on tangent continuity.
- Pressure advance should become cleaner mostly because extrusion flow is no
  longer split across many tiny generated moves.

Native arc pressure advance still requires correct extrusion mapping. The
planner must know:

- Total native arc path distance.
- Total extrusion distance.
- Extrusion ratio per path distance.
- Path velocity over time.

If XY motion is native but extrusion still assumes linear chord segments, PA can
become incorrect. Native arcs should therefore expose extrusion as a continuous
quantity over true arc length.

## Additional Research And Design Ideas

Native arcs should be implemented as the first instance of a more general curve
interface, not as a one-off special case. That keeps the work useful for future
native `G5`/`G5.1` support.

### Generic Curve Segment Interface

A planner-facing curve segment should expose geometry by path distance `s`:

```text
position(s)
tangent(s)
curvature(s)
length
start_tangent
end_tangent
e_ratio
max_velocity_bound
max_accel_bound
```

Then the planner can represent different G-code primitives with the same
interface:

```text
G1     -> LineSegment
G2/G3  -> ArcSegment
G5.1   -> QuadraticBezierSegment
G5     -> CubicBezierSegment
future -> NURBS or rational curves
```

This does not require implementing splines now. It only means the native `G2`/`G3`
object should use concepts that remain valid for splines: path distance,
tangent, curvature, and distance-based evaluation.

### Differential Motion Math

For any curve parameterized by path distance `s`:

```text
p_dot = T * v
p_ddot = T * a_t + N * kappa * v^2
```

Where:

- `T` is the unit tangent.
- `N` is the unit normal.
- `kappa` is curvature.
- `v = ds/dt` is path velocity.
- `a_t = d2s/dt2` is tangential acceleration.

Current linear motion effectively has:

```text
kappa = 0
p_ddot = T * a_t
```

Native arcs introduce the normal acceleration term:

```text
N * kappa * v^2
```

The conservative MVP can limit only by:

```text
v <= sqrt(max_accel / kappa)
```

For a flat circular arc this is:

```text
v <= sqrt(max_accel * radius)
```

A later implementation should reserve acceleration for the normal component and
reduce available tangential acceleration:

```text
sqrt(a_t^2 + (kappa * v^2)^2) <= max_accel
```

### Helical Arc Math

For a helix:

```text
x = cx + r * cos(theta)
y = cy + r * sin(theta)
z = z0 + h * theta
h = dz / angular_travel
```

The path length is:

```text
length = abs(angular_travel) * sqrt(r^2 + h^2)
```

The curvature is:

```text
kappa = r / (r^2 + h^2)
```

This is different from a flat arc's `1 / r`. A helical arc can have lower
curvature than a flat arc with the same planar radius because some path distance
is spent along the helical axis. The MVP can skip helical arcs, but the data
model should not assume curvature is always exactly `1 / radius`.

### Axis And Motor Limits

Axis velocity and acceleration vary continuously on a curve:

```text
axis_velocity_i = T_i * v
axis_accel_i = T_i * a_t + N_i * kappa * v^2
```

Axis constraints are:

```text
abs(axis_velocity_i) <= max_axis_velocity_i
abs(axis_accel_i) <= max_axis_accel_i
```

For equal X/Y limits on a simple Cartesian machine, the conservative curvature
cap may be sufficient for an MVP. For independent X/Y limits, CoreXY, bed mesh,
skew correction, and other transforms, this needs a more careful bound.

### Transform Problem

Native arcs interact poorly with the existing transform chain. Current transform
modules generally expect linear moves.

Examples:

- Bed mesh can add Z variation over XY, turning a planar arc into a warped 3D
  curve.
- Skew correction is an affine transform; a circle can become an ellipse.
- Exclude object and other modules may assume they can inspect or split linear
  moves.

Possible strategies:

1. Fallback to segmented arcs when a transform is active.
2. Require transforms to implement a curve-aware `move_arc()` path.
3. Promote transform handling to generic curve transforms.

The MVP should choose option 1. It preserves safety while native arcs are proven
on the untransformed motion path.

### Curvature Continuity

Native arcs remove fake segment junctions, but they do not automatically make all
motion fully smooth. A line-to-arc transition can be tangent-continuous while
still having a curvature jump:

```text
line curvature = 0
arc curvature = 1 / radius
```

That means acceleration can jump from zero normal acceleration to:

```text
v^2 / radius
```

Kalico already permits acceleration discontinuities in trapezoid profiles, so the
MVP can accept this. Later improvements could include:

- S-curve or jerk-limited path velocity profiles.
- Entry-speed reduction at large curvature changes.
- Optional blend tolerance similar to LinuxCNC `G64 P`.
- Clothoid or other transition curves that ramp curvature.

### Compatibility Beyond The MVP

Current `gcode_arcs.py` supports a narrow subset. Broader CNC compatibility would
eventually include:

- `G90.1` and `G91.1` arc center distance modes.
- Radius-format `R` arcs.
- Multi-turn arcs with a `P` word, where supported.
- Full circles.
- Helical arcs in selected planes.
- Arc radius consistency checks.

`R` arcs should not be first. Radius-format arcs are numerically sensitive near
semicircles and unsuitable for full circles in many controllers.

### Long-Term Spline Planner Direction

For arcs, curvature is constant or simple. For `G5`/`G5.1`, curvature varies
along the curve, so fixed trapezoid planning plus one global curvature cap becomes
less accurate.

The longer-term shape is path parameterization:

```text
s = path distance
v = ds/dt
a = d2s/dt2
```

With constraints that vary over the curve:

```text
velocity_limit(s)
accel_limit(s, v, a)
curvature_limit(s)
axis_limit(s, v, a)
```

TOPP-RA style planning solves this class of problem by computing reachable path
velocities under changing constraints. That is beyond the native-arc MVP, but the
native-arc design should keep the door open by using `s`, `tangent(s)`, and
`curvature(s)` as primary concepts.

## Likely Problem Areas

### Acceleration Accounting

The current trapezoid planner limits acceleration along a linear path. Native
arcs also need normal acceleration limits due to curvature.

### Independent Axis Limits

Kalico has support for independent X/Y velocity and acceleration limits. On an
arc, X/Y velocity and acceleration vary continuously, so the existing fixed
`axes_r` checks are not enough.

### Input Shaper

Input shaper code samples motion positions around a given time. It may continue
to work if `move_get_coord()` becomes curve-aware, but it needs explicit tests.

### Step Compression

Arc step timing may compress less efficiently than line step timing. Native arcs
could reduce host move count while increasing per-step timing irregularity.
Bandwidth and MCU queue behavior need measurement.

### Debug And History

`trapq_extract_old()` currently exports linear move data. Motion graph/debug
tools may need either curve metadata or sampled display-only output.

### Extrusion

For 3D printing, extrusion should map over total arc path length. For helical
arcs, this should use the true helical length, not only planar arc length.

### Compatibility

The existing implementation rejects `R` radius arcs. A native MVP can keep that
restriction and support only `I/J/K` center-offset arcs first.

The existing implementation also rejects relative coordinate mode. A native MVP
can keep that restriction while still supporting absolute and relative extrusion
semantics through `gcode_move.py`.

### Runtime Observability

Debugging native arcs will require sampled views of the path even though the
planner stores a curve. Existing debugging and history code that expects linear
move records may need display-only sampling or explicit curve metadata.

## Suggested Implementation Plan

### Stage 0: Document And Freeze Scope

Freeze the first native-arc target before writing planner code:

```text
G17 XY only
I/J center-offset only
absolute coordinate mode only
absolute and relative extrusion supported
no R arcs
no helix
no G18/G19
no transform-chain native support
segmented fallback remains
```

Example config direction:

```ini
[gcode_arcs]
native: true
resolution: 1.0
```

`resolution` remains useful for fallback mode and possibly debug sampling.

### Stage 1: Add Arc Geometry Library And Tests

Add a pure Python geometry helper before modifying `toolhead.py` or `trapq`.
This stage should be geometry-only. It should not queue moves, call the
toolhead, change lookahead, or modify C code.

Recommended location:

```text
klippy/extras/arc_geometry.py
```

Alternative location if the helper becomes planner-owned later:

```text
klippy/motion/arc_geometry.py
```

For the first stage, keeping it near `gcode_arcs.py` is acceptable because it
will initially replace the geometry math currently embedded in that module.

#### Stage 1 Public Object

Add a small immutable-style helper object, for example:

```python
class ArcGeometry:
    def __init__(self, start_pos, end_pos, offset, clockwise,
                 alpha_axis=0, beta_axis=1, helical_axis=2):
        ...
```

For the MVP, call it only with:

```text
alpha_axis = X
beta_axis = Y
helical_axis = Z
```

The constructor should accept positions in internal printer coordinates, not raw
G-code coordinates. This keeps G-code offsets, extrusion factor, and absolute
positioning outside the geometry helper.

The helper should compute:

- Center.
- Radius.
- Start angle.
- Signed angular travel.
- Path length.
- Planar path length.
- Linear helical-axis delta, even if MVP requires it to be zero.
- Start tangent.
- End tangent.
- Position at path distance.
- Normal at path distance.
- Curvature.
- `e_ratio = E_delta / path_length`.

Suggested stored fields:

```text
start_pos
end_pos
center
radius
clockwise
angular_travel
start_angle
end_angle
planar_length
linear_delta
path_length
start_tangent
end_tangent
curvature
e_delta
e_ratio
```

#### Stage 1 Formulas

Given current implementation-style `I/J` offsets:

```text
center_x = start_x + I
center_y = start_y + J
radius = hypot(I, J)
```

Start and target radius vectors:

```text
rs = (start_x - center_x, start_y - center_y)
rt = (end_x - center_x, end_y - center_y)
```

Angles:

```text
start_angle = atan2(rs_y, rs_x)
target_angle = atan2(rt_y, rt_x)
```

Unsigned CCW angular delta:

```text
delta = atan2(cross(rs, rt), dot(rs, rt))
if delta < 0:
    delta += 2*pi
```

Signed angular travel:

```text
if clockwise:
    angular_travel = delta - 2*pi
else:
    angular_travel = delta
```

Full-circle handling:

```text
if target planar position equals start planar position and angular_travel == 0:
    angular_travel = -2*pi if clockwise else 2*pi
```

Path lengths:

```text
planar_length = abs(radius * angular_travel)
linear_delta = end_pos[helical_axis] - start_pos[helical_axis]
path_length = hypot(planar_length, linear_delta)
```

For the MVP, reject nonzero `linear_delta` before using native mode. Still
computing it makes the helper ready for later helical arcs.

Flat arc curvature:

```text
curvature = 1 / radius
```

Later helical curvature:

```text
curvature = radius / (radius^2 + h^2)
h = linear_delta / angular_travel
```

#### Stage 1 Position Evaluation

The helper should support evaluation by path distance:

```python
def position_at(self, s):
    ...
```

For flat MVP arcs:

```text
u = s / path_length
theta = start_angle + angular_travel * u
x = center_x + radius * cos(theta)
y = center_y + radius * sin(theta)
z = start_z + linear_delta * u
e = start_e + e_delta * u
```

Clamp or validate `s`:

```text
0 <= s <= path_length
```

For tests, exact endpoint behavior matters. `position_at(path_length)` should
return the supplied target position, not a nearly equal trigonometric result.
This avoids tiny final-position drift.

#### Stage 1 Tangent Evaluation

Add:

```python
def tangent_at(self, s):
    ...
```

For a flat XY arc:

```text
theta = start_angle + angular_travel * (s / path_length)
dir = sign(angular_travel)
tx = -sin(theta) * dir
ty =  cos(theta) * dir
tz = 0
```

For helical readiness, tangent should eventually include the helical axis and be
normalized over true 3D path length:

```text
tangent = d(position) / ds
```

For MVP flat arcs, XY tangent length should be 1.0.

#### Stage 1 Validation Rules

The helper should reject or clearly report:

- Zero radius.
- Non-finite coordinates.
- Target radius mismatch above tolerance.
- Zero path length unless it is a valid full circle.
- Native MVP helix attempts if `linear_delta != 0`.

Radius mismatch check:

```text
start_radius = hypot(start_x - center_x, start_y - center_y)
target_radius = hypot(end_x - center_x, end_y - center_y)
abs(start_radius - target_radius) <= tolerance
```

Suggested tolerance:

```text
max(1e-6, radius * 1e-6)
```

The exact tolerance should be conservative and tested; too strict will reject
valid slicer output, while too loose can hide malformed arcs.

#### Stage 1 Test Matrix

Tests should cover:

- CW and CCW arcs.
- Endpoint angle selection.
- Radius consistency.
- Full-circle behavior.
- Tangents.
- Position sampling.
- Exact final position.
- Extrusion ratio over true path length.
- Radius mismatch rejection.
- Zero offset rejection.
- Path-distance midpoint sampling.

These tests should not depend on step generation.

Concrete test cases:

```text
Quarter CCW:
  start = (1, 0, 0, 0)
  end   = (0, 1, 0, 0)
  I/J   = (-1, 0)
  angular_travel = pi/2
  length = pi/2
  midpoint = (sqrt(1/2), sqrt(1/2))

Quarter CW:
  start = (0, 1, 0, 0)
  end   = (1, 0, 0, 0)
  I/J   = (0, -1)
  angular_travel = -pi/2
  length = pi/2

Full CCW circle:
  start = end = (1, 0, 0, 0)
  I/J = (-1, 0)
  angular_travel = 2*pi
  length = 2*pi

Extruding half circle:
  start_e = 10
  end_e = 14
  radius = 2
  angular_travel = pi
  e_ratio = 4 / (2*pi)
```

#### Stage 1 Done Criteria

Stage 1 is complete when:

- The geometry helper is independent of printer objects.
- Unit tests pass without loading a printer config.
- The existing segmented `gcode_arcs.py` behavior is unchanged.
- The helper can reproduce current endpoint and full-circle behavior.
- The helper exposes enough data for Stage 2 `ArcMove`.

### Stage 2: Introduce Generic Curve Concepts In Python

Add the planner-facing concept of a curve move, even if the only implementation
is `ArcMove`.

The object should expose:

- `move_d`
- `start_pos`
- `end_pos`
- `axes_d`
- `axes_r[3]` as extrusion ratio compatibility
- `start_tangent`
- `end_tangent`
- `curvature`
- `max_cruise_v2`
- `delta_v2`
- `smooth_delta_v2`
- `set_junction()`
- `timing_callbacks`

Line moves can be adapted to expose `start_tangent == end_tangent == axes_r[:3]`.
Arc moves derive tangents from the arc geometry.

### Stage 3: Refactor Lookahead To Use Tangents

Refactor junction calculation so it uses:

- Previous move end tangent.
- Current move start tangent.

Linear moves should produce identical behavior to the current implementation.
This is an important regression requirement.

Apply a conservative curvature speed cap to native arcs before final trapezoid
timing:

```text
arc_max_v = sqrt(max_accel / kappa)
```

For flat circles:

```text
arc_max_v = sqrt(max_accel * radius)
```

### Stage 4: Add Transform Fallback Detection

Native arcs should not silently bypass move transforms. For the MVP, if a
non-toolhead move transform is active, native arcs should fall back to the
existing segmented implementation.

Later, transform modules can opt into curve support with a dedicated interface.

### Stage 5: Extend trapq

Extend the C trap queue without breaking linear moves.

Required work:

- Add move type metadata to `struct move`, or introduce a compatible curve move
  layout.
- Add `trapq_append_arc()`.
- Update CFFI declarations.
- Update `move_get_coord()` to dispatch line vs arc.
- Keep `trapq_append()` unchanged for linear moves and extruder trapq use.

```text
line: start + axes_r * distance
arc: evaluate arc at distance
```

The goal is to preserve existing kinematics helpers that call
`move_get_coord()`.

### Stage 6: Step Generation

Verify `itersolve_generate_steps()` continues to work with curve-aware
`move_get_coord()`. The iterative solver should not need to know the move type
directly if each kinematic callback receives correct coordinates.

Required audit:

- `itersolve.c::check_active()`
- `kin_shaper.c` direct `axes_r` reads
- `trapq_extract_old()` motion reporting behavior
- Cartesian step generation
- CoreXY step generation
- At least one non-Cartesian kinematic callback

### Stage 7: Wire Native `G2`/`G3` End To End

Connect native mode from `gcode_arcs.py` through `gcode_move.py` into
`toolhead.py`. The normal segmented path should still be selected when native
mode is disabled.

The native path should:

- Preserve feedrate state.
- Preserve extrusion mode.
- Update final G-code position.
- Call the same pause/check/stall paths as normal moves.
- Expose correct E ratio to extruder checks and PA.

Current implementation:

- `[gcode_arcs] native: true` enables the experimental native path.
- Native mode is limited to non-helical XY-plane `G2`/`G3` arcs.
- Unsupported planes, helical arcs, disabled native mode, invalid geometry, and
  active move transforms fall back to the existing segmented `G1` path.
- `gcode_arcs.py` detects eligible arcs and calls `GCodeMove.cmd_G2G3()`.
- `GCodeMove.cmd_G2G3()` preserves normal G-code state handling for `F` and
  `E`, constructs `ArcGeometry`, then updates `last_position` only after the
  arc geometry is accepted.
- `ToolHead.move_arc()` wraps the geometry in `ArcMove`, runs the existing
  kinematic and extruder checks, updates `commanded_pos`, and queues the move
  through lookahead.
- `ToolHead._process_moves()` appends `ArcMove` instances to trapq with
  `trapq_append_arc()` while keeping ordinary line moves on `trapq_append()`.

Stage 7 tests:

- `test/test_gcode_arcs_native.py`
- `test/test_gcode_move_transforms.py`
- Existing geometry, curve move, trapq arc, and import tests.

### Stage 8: Extruder And Pressure Advance Validation

Keep extrusion synchronized over the native arc's total path length and timing.
For the first native implementation, match current `G1` semantics for absolute
and relative extrusion.

Validate:

- `PrinterExtruder.check_move()` limits still behave correctly.
- `PrinterExtruder.calc_junction()` sees correct E ratio changes.
- Pressure advance does not react to internal fake chord boundaries.
- `find_past_position()` remains correct for sensors and MPC code that query
  extruder position history.

Current implementation notes:

- `ArcMove.axes_r[3]` is the E-per-path ratio, so extruder velocity and
  acceleration scale from native arc path length instead of endpoint chord
  length.
- `ArcMove.has_xy_motion` marks arcs as real XY moves even when the start and
  end XY coordinates are identical, such as full-circle arcs.
- `PrinterExtruder.check_move()` and `PrinterExtruder.move()` now use the
  motion flag in addition to endpoint delta checks. This prevents native
  full-circle arcs with extrusion from being misclassified as extrude-only
  moves.
- Pressure advance eligibility is still based on positive extrusion over XY
  motion. Native arcs provide one continuous extruder trapq move, so PA sees
  the arc as one move instead of many chord boundaries.
- Extruder trapq remains linear in E over the native arc's path length; the
  XY arc geometry lives in the toolhead trapq.

Stage 8 tests:

- `test/test_arc_extrusion.py`
- Existing curve move, native G-code arc, trapq arc, transform, and import
  tests.

### Stage 9: Testing And Performance

Add regression tests for:

- CW and CCW arcs.
- Exact endpoint behavior.
- Full-circle behavior.
- Line-to-arc and arc-to-line tangent junctions.
- Small-radius curvature speed limiting.
- Transform fallback.
- Input shaper enabled.
- Cartesian and CoreXY step generation.
- Step compression size and buffer behavior.
- Fallback segmented mode.

Measure native versus segmented arcs:

- Host move count.
- Host CPU during planning.
- Step generation CPU.
- Stepcompress output size.
- MCU queue pressure.
- Final path accuracy.
- Junction speed behavior.

### Stage 10: Broaden Arc Compatibility

After XY center-offset arcs are stable:

- Add `G18` XZ and `G19` YZ planes.
- Add helical arcs with correct helical length and curvature.
- Add `G90.1` and `G91.1` arc center distance modes if desired.
- Consider `R` arcs after robust center-offset arcs.
- Consider multi-turn arcs if desired.

### Stage 11: Prepare For Native `G5`/`G5.1`

Once native arcs are stable, reuse the curve interface for splines.

Spline-specific follow-up work:

- Arc-length parameterization.
- Variable curvature sampling or bounds.
- Tangent and curvature continuity at junctions.
- Path-parameterized velocity planning.
- Optional TOPP-RA style reachability planning for variable constraints.

### Stage 12: Optional Blend/Tolerance Model

Consider a LinuxCNC-like exact/blended path mode after exact native arcs work.

Possible config shape:

```ini
[curve_motion]
path_tolerance: 0.02
curve_blending: true
```

This should be separate from the exact native-arc MVP. Exact native arcs preserve
the requested path; blending intentionally trades path accuracy for smoother or
faster motion.

## Research References

- LinuxCNC G-code documentation, especially `G2/G3`, `G64`, and arc center modes:
  https://linuxcnc.org/docs/2.8/html/gcode/g-code.html
- LinuxCNC user concepts for trajectory control and path blending:
  https://www.linuxcnc.org/docs/html/user/user-concepts.html
- Grbl planner source and comments on short segmented moves:
  https://raw.githubusercontent.com/gnea/grbl/master/grbl/planner.c
- Klipper kinematics documentation:
  https://www.klipper3d.org/Kinematics.html
- Klipper pressure advance documentation:
  https://www.klipper3d.org/Pressure_Advance.html
- TOPP-RA paper:
  https://www.researchgate.net/publication/318671280_A_New_Approach_to_Time-Optimal_Path_Parameterization_Based_on_Reachability_Analysis

## Recommendation

Native `G2`/`G3` should be implemented before native `G5`/`G5.1`.
Arcs have exact length, exact tangents, and constant curvature, so they are the
lowest-risk path to introducing non-linear motion primitives into Kalico's
planner. Once native arcs work, splines become a follow-up problem of variable
curvature and arc-length parameterization rather than a full planner rewrite
from scratch.
