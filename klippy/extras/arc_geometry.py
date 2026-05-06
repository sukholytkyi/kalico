# Native arc geometry helpers
#
# Copyright (C) 2026  Kalico contributors
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math


class ArcGeometryError(Exception):
    pass


class ArcGeometry:
    def __init__(
        self,
        start_pos,
        end_pos,
        offset,
        clockwise,
        alpha_axis=0,
        beta_axis=1,
        helical_axis=2,
        allow_helical=False,
        tolerance=None,
    ):
        self.start_pos = tuple(float(v) for v in start_pos)
        self.end_pos = tuple(float(v) for v in end_pos)
        self.offset = tuple(float(v) for v in offset)
        self.clockwise = bool(clockwise)
        self.alpha_axis = alpha_axis
        self.beta_axis = beta_axis
        self.helical_axis = helical_axis
        if len(self.start_pos) < 4 or len(self.end_pos) < 4:
            raise ArcGeometryError("Arc positions must include XYZE coordinates")
        if len(self.offset) != 2:
            raise ArcGeometryError("Arc offset must contain two planar coordinates")
        self._check_finite()

        start_a = self.start_pos[alpha_axis]
        start_b = self.start_pos[beta_axis]
        end_a = self.end_pos[alpha_axis]
        end_b = self.end_pos[beta_axis]
        off_a, off_b = self.offset
        if not off_a and not off_b:
            raise ArcGeometryError("Arc requires a non-zero center offset")

        center_a = start_a + off_a
        center_b = start_b + off_b
        self.center = (center_a, center_b)

        start_radius_a = start_a - center_a
        start_radius_b = start_b - center_b
        target_radius_a = end_a - center_a
        target_radius_b = end_b - center_b
        self.radius = math.hypot(start_radius_a, start_radius_b)
        if not self.radius:
            raise ArcGeometryError("Arc radius must be non-zero")

        target_radius = math.hypot(target_radius_a, target_radius_b)
        self.tolerance = (
            max(1.0e-6, self.radius * 1.0e-6)
            if tolerance is None
            else float(tolerance)
        )
        if abs(self.radius - target_radius) > self.tolerance:
            raise ArcGeometryError(
                "Arc endpoint radius mismatch: %.12g vs %.12g"
                % (self.radius, target_radius)
            )

        self.start_angle = math.atan2(start_radius_b, start_radius_a)
        cross = start_radius_a * target_radius_b - start_radius_b * target_radius_a
        dot = start_radius_a * target_radius_a + start_radius_b * target_radius_b
        delta = math.atan2(cross, dot)
        if delta < 0.0:
            delta += 2.0 * math.pi
        if self.clockwise:
            self.angular_travel = delta - 2.0 * math.pi
        else:
            self.angular_travel = delta

        if (
            abs(end_a - start_a) <= self.tolerance
            and abs(end_b - start_b) <= self.tolerance
            and abs(self.angular_travel) <= self.tolerance
        ):
            self.angular_travel = -2.0 * math.pi if self.clockwise else 2.0 * math.pi

        self.end_angle = self.start_angle + self.angular_travel
        self.planar_length = abs(self.radius * self.angular_travel)
        self.linear_delta = self.end_pos[helical_axis] - self.start_pos[helical_axis]
        if not allow_helical and abs(self.linear_delta) > self.tolerance:
            raise ArcGeometryError("Native arc MVP does not support helical arcs")
        self.path_length = math.hypot(self.planar_length, self.linear_delta)
        if not self.path_length:
            raise ArcGeometryError("Arc path length must be non-zero")

        if self.linear_delta:
            h = self.linear_delta / self.angular_travel
            self.curvature = self.radius / (self.radius * self.radius + h * h)
        else:
            self.curvature = 1.0 / self.radius
        self.e_delta = self.end_pos[3] - self.start_pos[3]
        self.e_ratio = self.e_delta / self.path_length
        self.start_tangent = self.tangent_at(0.0)
        self.end_tangent = self.tangent_at(self.path_length)

    def _check_finite(self):
        values = list(self.start_pos) + list(self.end_pos) + list(self.offset)
        if not all(math.isfinite(v) for v in values):
            raise ArcGeometryError("Arc coordinates must be finite")

    def _fraction_at(self, distance):
        distance = float(distance)
        if distance < -self.tolerance or distance > self.path_length + self.tolerance:
            raise ArcGeometryError("Arc distance is outside the move")
        if distance <= 0.0:
            return 0.0
        if distance >= self.path_length:
            return 1.0
        return distance / self.path_length

    def position_at(self, distance):
        u = self._fraction_at(distance)
        if u == 0.0:
            return self.start_pos
        if u == 1.0:
            return self.end_pos
        theta = self.start_angle + self.angular_travel * u
        pos = list(self.start_pos)
        pos[self.alpha_axis] = self.center[0] + self.radius * math.cos(theta)
        pos[self.beta_axis] = self.center[1] + self.radius * math.sin(theta)
        pos[self.helical_axis] = (
            self.start_pos[self.helical_axis] + self.linear_delta * u
        )
        pos[3] = self.start_pos[3] + self.e_delta * u
        return tuple(pos)

    def tangent_at(self, distance):
        u = self._fraction_at(distance)
        theta = self.start_angle + self.angular_travel * u
        direction = -1.0 if self.angular_travel < 0.0 else 1.0
        planar_scale = self.planar_length / self.path_length
        tangent = [0.0, 0.0, 0.0, 0.0]
        tangent[self.alpha_axis] = -math.sin(theta) * direction * planar_scale
        tangent[self.beta_axis] = math.cos(theta) * direction * planar_scale
        tangent[self.helical_axis] = self.linear_delta / self.path_length
        return tuple(tangent)

    def normal_at(self, distance):
        u = self._fraction_at(distance)
        theta = self.start_angle + self.angular_travel * u
        normal = [0.0, 0.0, 0.0, 0.0]
        normal[self.alpha_axis] = -math.cos(theta)
        normal[self.beta_axis] = -math.sin(theta)
        return tuple(normal)
