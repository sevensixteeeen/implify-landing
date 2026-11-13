"""Barometric altitude; complementary filter; dead reckoning and the F1 disagreement detector.

ARCHITECTURE section 6.2. The avionics model assumes constant gravity g0 and the layer-1
standard-atmosphere pressure law. It deliberately differs from the truth model.

Disagreement detector (F1 Policy B, docs/F1_DECISIONS.md sections 6b and 6e). Uses avionics data
only (FR-AVI-01): barometer samples, an accelerometer-only dead reckoning, the filter's vertical
speed, the pad calibration and the avionics configuration. Its only output is a latched flag, which
the flight computer turns into an UNRELIABLE estimate and the state machine into the F1-D4
deployment restriction. It never commands deployment and never changes flight state.

    q_D,k = |dD_k| / S_k
    S_k   = sqrt( (2/5) sigma_h,k^2 + sigma_DR(t_k)^2 + (kappa |v_hat,k| dt_blocks)^2 )

dD_k is the mean of D over the latest 5 valid barometer samples minus the mean over samples
k-10 .. k-14, with D = barometric altitude - dead-reckoning altitude. The flag is raised after
PERSISTENCE consecutive defined values above THRESHOLD, or at once (UNCHECKABLE) when the dead
reckoning becomes invalid. Once raised it is never cleared.
"""
import math
from collections import deque
from dataclasses import dataclass
from enum import StrEnum

from aerosat.avionics.calibration import CalibrationResult
from aerosat.core.constants import G0_MPS2, LAPSE_RATE_KPM, R_AIR_JPKGK

_BARO_EXPONENT = R_AIR_JPKGK * LAPSE_RATE_KPM / G0_MPS2  # dimensionless, about 0.190263


def baro_altitude_agl_m(pressure_pa: float, reference_pressure_pa: float, reference_temperature_k: float) -> float:
    """Altitude above the pad (m) from a pressure reading, inverting the layer-1 pressure law.

    h_baro = (T_ref / L) * (1 - (p / p_ref)^(R*L/g0))
    """
    return reference_temperature_k / LAPSE_RATE_KPM * (1.0 - (pressure_pa / reference_pressure_pa) ** _BARO_EXPONENT)


class ComplementaryFilter:
    """Blends fast-but-drifting accelerometer integration with slow-but-stable barometric altitude.

    `gain_h` is dimensionless; `gain_v_hz` is in 1/s. Both are applied once per barometer sample.
    """

    def __init__(self, gain_h: float, gain_v_hz: float, period_s: float) -> None:
        self._gain_h = gain_h
        self._gain_v_hz = gain_v_hz
        self._period_s = period_s
        self.altitude_agl_m = 0.0
        self.velocity_mps = 0.0

    def predict(self, acceleration_mps2: float) -> None:
        """Advance one avionics period using the estimated vertical acceleration."""
        period_s = self._period_s
        self.altitude_agl_m += self.velocity_mps * period_s + 0.5 * acceleration_mps2 * period_s * period_s
        self.velocity_mps += acceleration_mps2 * period_s

    def correct(self, baro_altitude_m: float) -> None:
        """Pull the estimate toward a barometric altitude."""
        residual_m = baro_altitude_m - self.altitude_agl_m
        self.altitude_agl_m += self._gain_h * residual_m
        self.velocity_mps += self._gain_v_hz * residual_m


def estimated_acceleration_mps2(specific_force_z_mps2: float, accel_bias_mps2: float) -> float:
    """Vertical kinematic acceleration: bias-corrected specific force minus g0. About 0 at rest."""
    return (specific_force_z_mps2 - accel_bias_mps2) - G0_MPS2


# ------------------------------------------------------------------ F1 disagreement detector

THRESHOLD = 7.0
"""Alarm threshold on q_D, from the noise derivation (F1_DECISIONS 6e.1). Fixed by design."""
PERSISTENCE = 3
"""Consecutive defined values above THRESHOLD that raise the flag (6e.1). Fixed by design."""
BLOCK = 5
"""Samples per block mean."""
WINDOW = 15
"""Contiguous valid barometer samples needed for one dD value (blocks k..k-4 and k-10..k-14)."""


class FlagReason(StrEnum):
    DISAGREEMENT = "DISAGREEMENT"
    """q_D exceeded the threshold on PERSISTENCE consecutive samples."""
    UNCHECKABLE = "UNCHECKABLE"
    """The dead reckoning became invalid, so the sensors can no longer be compared."""


@dataclass(frozen=True, slots=True)
class DisagreementFlag:
    tick: int
    """Avionics tick at which the flag was raised."""
    reason: FlagReason
    q: float | None
    """q_D on that tick (None for UNCHECKABLE)."""

    def describe(self) -> str:
        if self.reason is FlagReason.UNCHECKABLE:
            return f"UNCHECKABLE disagreement flag raised at tick {self.tick} (accelerometer dead reckoning invalid)"
        return f"DISAGREEMENT flag raised at tick {self.tick} (q_D = {self.q:.2f} > {THRESHOLD})"


class DeadReckoning:
    """Accelerometer-only altitude and velocity from the end of pad calibration (6b). Never uses the barometer."""

    def __init__(self, period_s: float) -> None:
        self._dt = period_s
        self.altitude_m = 0.0
        self.velocity_mps = 0.0
        self.valid = True

    def step(self, acceleration_mps2: float) -> None:
        """Advance one avionics period with the (possibly held) gravity-compensated acceleration."""
        if not self.valid:
            return
        dt = self._dt
        self.altitude_m += self.velocity_mps * dt + 0.5 * acceleration_mps2 * dt * dt
        self.velocity_mps += acceleration_mps2 * dt

    def invalidate(self) -> None:
        """The acceleration hold has expired. The dead reckoning stays invalid for the rest of the flight."""
        self.valid = False


def baro_altitude_sigma_m(pressure_pa: float, sigma_p_pa: float, calibration: CalibrationResult) -> float:
    """sigma_h: barometer noise mapped to altitude through the slope of the avionics pressure law at `pressure_pa`."""
    n = _BARO_EXPONENT
    p_ref = calibration.reference_pressure_pa
    slope = calibration.reference_temperature_k / LAPSE_RATE_KPM * n * (pressure_pa / p_ref) ** (n - 1.0) / p_ref
    return sigma_p_pa * abs(slope)


def dead_reckoning_velocity_sigma_mps(t_s: float, sigma_a_mps2: float, calibration_samples: int, period_s: float) -> float:
    """sigma_v,DR(t): calibration bias error plus integrated accelerometer noise, t seconds after calibration."""
    t = max(t_s, 0.0)
    bias = sigma_a_mps2 * t / math.sqrt(max(calibration_samples, 1))
    return math.sqrt(bias * bias + sigma_a_mps2 * sigma_a_mps2 * period_s * t)


def disagreement_statistic(dd_m: float, sigma_h_m: float, sigma_dr_m: float, kappa: float,
                           v_hat_mps: float, dt_blocks_s: float) -> float:
    """q_D for one dD value. The speed term enters only through the denominator (F1-D3)."""
    spread = math.sqrt(0.4 * sigma_h_m * sigma_h_m + sigma_dr_m * sigma_dr_m + (kappa * abs(v_hat_mps) * dt_blocks_s) ** 2)
    return abs(dd_m) / spread


class DisagreementMonitor:
    """Policy B (F1_DECISIONS 6e.1, 6e.2): q_D, persistence, latched flag."""

    def __init__(self, sigma_p_pa: float, sigma_a_mps2: float, kappa: float, calibration: CalibrationResult,
                 period_s: float) -> None:
        self._sigma_p = sigma_p_pa
        self._sigma_a = sigma_a_mps2
        self._kappa = kappa
        self._calibration = calibration
        self._period_s = period_s
        self._window: deque[tuple[int, float]] = deque(maxlen=WINDOW)
        self._count = 0
        self.flag: DisagreementFlag | None = None
        self.last_q: float | None = None

    def barometer_rejected(self) -> None:
        """A rejected barometer sample breaks contiguity: the next dD needs 15 new valid samples."""
        self._window.clear()
        self._count = 0
        self.last_q = None

    def uncheckable(self, tick: int) -> None:
        """The dead reckoning is invalid: the sensors cannot be compared. Raises the flag (6e.1)."""
        if self.flag is None:
            self.flag = DisagreementFlag(tick, FlagReason.UNCHECKABLE, None)

    def barometer_sample(self, tick: int, baro_altitude_m: float, pressure_pa: float, dr_altitude_m: float,
                         t_since_calibration_s: float, v_hat_mps: float) -> float | None:
        """Add one valid barometer sample. Returns q_D, or None while dD is undefined."""
        self._window.append((tick, baro_altitude_m - dr_altitude_m))
        if len(self._window) < WINDOW:
            self.register(tick, None)
            return None
        w = self._window
        newer = sum(w[i][1] for i in range(WINDOW - BLOCK, WINDOW)) / BLOCK
        older = sum(w[i][1] for i in range(BLOCK)) / BLOCK
        dt_blocks_s = (w[WINDOW - 3][0] - w[2][0]) / 1000.0          # centre of k..k-4 minus centre of k-10..k-14
        sigma_h = baro_altitude_sigma_m(pressure_pa, self._sigma_p, self._calibration)
        sigma_dr = dt_blocks_s * dead_reckoning_velocity_sigma_mps(
            t_since_calibration_s, self._sigma_a, self._calibration.accel_sample_count, self._period_s)
        q = disagreement_statistic(newer - older, sigma_h, sigma_dr, self._kappa, v_hat_mps, dt_blocks_s)
        self.register(tick, q)
        return q

    def register(self, tick: int, q: float | None) -> None:
        """Persistence: PERSISTENCE consecutive defined values above THRESHOLD raise the flag. None resets."""
        self.last_q = q
        if self.flag is not None:
            return
        self._count = self._count + 1 if (q is not None and q > THRESHOLD) else 0
        if self._count >= PERSISTENCE:
            self.flag = DisagreementFlag(tick, FlagReason.DISAGREEMENT, q)
