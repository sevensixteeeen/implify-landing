"""Thrust table, impulse, mass, drag area.

ARCHITECTURE section 4.7. All vehicle numbers are fictional simulation parameters.
"""
import math
from bisect import bisect_right
from collections.abc import Sequence
from dataclasses import dataclass, field


class _PiecewiseLinear:
    """A piecewise-linear table over time, with its exact (trapezoid) integral.

    Breakpoints are given in integer milliseconds so that every slope change lands on a tick.
    """

    __slots__ = ("times_s", "values", "_cumulative")

    def __init__(self, points: Sequence[tuple[int, float]]) -> None:
        for t_ms, _ in points:
            if isinstance(t_ms, bool) or not isinstance(t_ms, int):
                raise ValueError(f"breakpoint times must be integer milliseconds, got {t_ms!r}")
        if any(later[0] <= earlier[0] for earlier, later in zip(points, points[1:])):
            raise ValueError("breakpoint times must be strictly increasing")
        self.times_s = tuple(t_ms / 1000 for t_ms, _ in points)
        self.values = tuple(float(value) for _, value in points)
        cumulative = [0.0]
        for i in range(1, len(points)):
            width_s = self.times_s[i] - self.times_s[i - 1]
            cumulative.append(cumulative[-1] + (self.values[i - 1] + self.values[i]) / 2 * width_s)
        self._cumulative = tuple(cumulative)

    def _inside(self, t_s: float) -> bool:
        return bool(self.times_s) and self.times_s[0] <= t_s <= self.times_s[-1]

    def _value_inside(self, t_s: float) -> float:
        i = min(bisect_right(self.times_s, t_s) - 1, len(self.times_s) - 2)
        if i < 0:  # a single breakpoint
            return self.values[0]
        t0, t1 = self.times_s[i], self.times_s[i + 1]
        return self.values[i] + (self.values[i + 1] - self.values[i]) * (t_s - t0) / (t1 - t0)

    def _integral_inside(self, t_s: float) -> float:
        i = min(bisect_right(self.times_s, t_s) - 1, len(self.times_s) - 2)
        if i < 0:
            return 0.0
        return self._cumulative[i] + (self.values[i] + self._value_inside(t_s)) / 2 * (t_s - self.times_s[i])


class ThrustCurve(_PiecewiseLinear):
    """Thrust (N) against time since ignition. Zero outside the table."""

    def thrust_n(self, t_since_ignition_s: float) -> float:
        return self._value_inside(t_since_ignition_s) if self._inside(t_since_ignition_s) else 0.0

    def impulse_ns(self, t_since_ignition_s: float) -> float:
        """Impulse delivered up to a time since ignition, N*s. Exact for the piecewise-linear table."""
        if not self.times_s or t_since_ignition_s <= self.times_s[0]:
            return 0.0
        if t_since_ignition_s >= self.times_s[-1]:
            return self._cumulative[-1]
        return self._integral_inside(t_since_ignition_s)

    @property
    def total_impulse_ns(self) -> float:
        return self._cumulative[-1]

    @property
    def burn_duration_s(self) -> float:
        """Time since ignition of the last breakpoint, s. Zero for an empty table."""
        return self.times_s[-1] if self.times_s else 0.0


class RollProfile(_PiecewiseLinear):
    """Scripted roll rate (rad/s) against time since scenario start.

    Kinematic only: prescribed, not computed from torques. The first value is held before the
    first point and the last value after the last point. An empty profile means no roll.
    """

    def roll_rate_radps(self, t_s: float) -> float:
        if not self.times_s:
            return 0.0
        if t_s <= self.times_s[0]:
            return self.values[0]
        if t_s >= self.times_s[-1]:
            return self.values[-1]
        return self._value_inside(t_s)

    def roll_angle_rad(self, t_s: float) -> float:
        """Roll angle accumulated since scenario start, rad. Exact integral of the rate."""
        if not self.times_s or t_s <= 0.0:
            return 0.0
        first_s, last_s = self.times_s[0], self.times_s[-1]
        angle_rad = self.values[0] * min(t_s, first_s)
        if t_s > first_s:
            angle_rad += self._integral_inside(min(t_s, last_s))
        if t_s > last_s:
            angle_rad += self.values[-1] * (t_s - last_s)
        return angle_rad


@dataclass(frozen=True)
class Vehicle:
    """Mass, thrust and drag area of the (fictional) vehicle.

    `cd_area_m2` is drag coefficient times reference area. Propellant is consumed in proportion to
    delivered impulse. A vehicle without a thrust table keeps its initial mass.

    Validation is strict: every value must be a finite number, the dry mass must be positive, and
    the propellant mass and drag area must not be negative. Zero propellant is valid.
    """

    dry_mass_kg: float
    propellant_mass_kg: float
    cd_area_m2: float
    thrust_curve: ThrustCurve = field(default_factory=lambda: ThrustCurve(()))

    def __post_init__(self) -> None:
        for name in ("dry_mass_kg", "propellant_mass_kg", "cd_area_m2"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f"{name} must be a finite number, got {value!r}")
        if self.dry_mass_kg <= 0.0:
            raise ValueError("dry_mass_kg must be positive")
        if self.propellant_mass_kg < 0.0:
            raise ValueError("propellant_mass_kg must not be negative")
        if self.cd_area_m2 < 0.0:
            raise ValueError("cd_area_m2 must not be negative")

    def thrust_n(self, t_since_ignition_s: float) -> float:
        return self.thrust_curve.thrust_n(t_since_ignition_s)

    def mass_kg(self, t_since_ignition_s: float) -> float:
        total_ns = self.thrust_curve.total_impulse_ns
        if total_ns <= 0.0:
            return self.dry_mass_kg + self.propellant_mass_kg
        burned_fraction = self.thrust_curve.impulse_ns(t_since_ignition_s) / total_ns
        return self.dry_mass_kg + self.propellant_mass_kg * (1.0 - burned_fraction)
