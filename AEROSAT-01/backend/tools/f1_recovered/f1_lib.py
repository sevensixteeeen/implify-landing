"""F1 experiments E1-E7: in-memory instrumentation only. Nothing in the repository is written.

One closed-loop flight = simulator -> sensors -> (optional injected fault) -> FlightComputer, wired
exactly like tests/unit/avionics/conftest.py::run_flight. The filter's correct() is wrapped on the
instance to capture the innovation. Candidate monitors are computed OFFLINE from the logged
streams; they never feed back into the avionics.
"""
import copy
import math
import os
import sys
from dataclasses import replace

BACKEND = r"C:\Users\Harshit\alacrity\AEROSAT-01\backend"
sys.path.insert(0, os.path.join(BACKEND, "tests", "unit", "avionics"))
sys.path.insert(0, os.path.join(BACKEND, "src"))
os.environ.setdefault("PYTHONDONTWRITEBYTECODE", "1")
sys.dont_write_bytecode = True

import conftest as c  # noqa: E402
from aerosat.avionics.estimator import _BARO_EXPONENT, baro_altitude_agl_m  # noqa: E402
from aerosat.avionics.flight_computer import FlightComputer, load_avionics_config  # noqa: E402
from aerosat.core.constants import G0_MPS2, LAPSE_RATE_KPM  # noqa: E402
from aerosat.sensors.suite import SensorSuite, suite_config_from_scenario  # noqa: E402
from aerosat.sim.simulator import Simulator  # noqa: E402

PERIOD = 5
T = 0.005
BARO_DT = 20
WINDOWS_S = (0.2, 0.5, 1.0, 2.0)
PHASES = ("PAD", "BOOST", "COAST", "NEAR_APOGEE", "DESCENT", "LANDED")
NEAR_APOGEE_MS = 2000


# ------------------------------------------------------------------ scenarios
def scenario(name, *, ignition_ms=None):
    s = copy.deepcopy(c.REF01)
    if name == "zero_drag":
        s["vehicle"]["drag_coefficient"] = 0.0
        s["avionics"]["expected_coast_duration_ms"] = 23665
        s["avionics"]["backup_deployment_delay_ms"] = 26000
    elif name == "low_thrust":
        s["vehicle"]["thrust_curve"] = [{"t_ms": 0, "thrust_n": 0.0}, {"t_ms": 100, "thrust_n": 280.0},
                                        {"t_ms": 16000, "thrust_n": 280.0}, {"t_ms": 16200, "thrust_n": 0.0}]
    elif name != "ref01":
        raise ValueError(name)
    if ignition_ms is not None:
        s["timing"]["t_ignition_ms"] = ignition_ms
    return s


# ------------------------------------------------------------------ faults (sensor-sample level)
def _p_from_h(h, cal):
    base = 1.0 - h * LAPSE_RATE_KPM / cal.reference_temperature_k
    return cal.reference_pressure_pa * base ** (1.0 / _BARO_EXPONENT)


class Fault:
    """kind: baro_offset (altitude offset m), baro_drift (m/s), baro_frozen, baro_invalid,
    accel_value (stuck at value), accel_invalid. Active for t0 <= tick < t1."""

    def __init__(self, kind, t0, t1=None, value=0.0):
        self.kind, self.t0, self.t1, self.value = kind, t0, (t1 if t1 is not None else 10**12), value
        self.held = None

    def active(self, tick):
        return self.t0 <= tick < self.t1

    def apply(self, tick, samples, computer):
        out = []
        for s in samples:
            sid = s.sensor_id
            if sid == "baro":
                if not self.kind.startswith("baro"):
                    out.append(s); continue
                if not self.active(tick):
                    if self.kind == "baro_frozen" and tick < self.t0:
                        self.held = s.value
                    out.append(s); continue
                cal = computer.calibration
                if self.kind == "baro_frozen":
                    out.append(replace(s, value=self.held)); continue
                if self.kind == "baro_invalid":
                    out.append(replace(s, saturated=True)); continue
                if self.kind == "baro_pa":
                    out.append(replace(s, value=s.value + self.value)); continue
                h = baro_altitude_agl_m(s.value, cal.reference_pressure_pa, cal.reference_temperature_k)
                dh = self.value if self.kind == "baro_offset" else self.value * (tick - self.t0) / 1000.0
                out.append(replace(s, value=_p_from_h(h + dh, cal)))
            elif sid == "accel" and s.channel == "z" and self.kind == "accel_hold" and not self.active(tick):
                if tick < self.t0:
                    self.held = s.value
                out.append(s)
            elif sid == "accel" and s.channel == "z" and self.kind.startswith("accel") and self.active(tick):
                if self.kind == "accel_hold":
                    out.append(replace(s, value=self.held))
                elif self.kind == "accel_value":
                    out.append(replace(s, value=self.value))
                else:
                    out.append(replace(s, saturated=True))
            else:
                out.append(s)
        return out


class Faults:
    def __init__(self, *faults):
        self.faults = faults

    def apply(self, tick, samples, computer):
        for f in self.faults:
            samples = f.apply(tick, samples, computer)
        return samples

    @property
    def onset(self):
        return min(f.t0 for f in self.faults)


# ------------------------------------------------------------------ one flight
def fly(scen, seed, *, ideal=False, fault=None, stop_tick=None):
    av = load_avionics_config(scen)
    sensors = copy.deepcopy(scen["sensors"]); sensors["ideal"] = ideal
    sim = Simulator(c.ref01_simulator_config(scen))
    suite = SensorSuite(suite_config_from_scenario(sensors), master_seed=seed)
    comp = FlightComputer(av)
    filt = comp._filter
    box = [None]
    orig = filt.correct

    def correct(h):
        box[0] = h - filt.altitude_agl_m
        orig(h)
    filt.correct = correct

    rows, events, commands = [], [], []
    state, pending = sim.state, ()
    while True:
        samples = suite.sample(state)
        if fault is not None and comp.calibration is not None:
            samples = fault.apply(state.tick, samples, comp)
        elif fault is not None:
            # before calibration ends: only record the frozen value
            for f in fault.faults:
                if f.kind == "baro_frozen":
                    for s in samples:
                        if s.sensor_id == "baro":
                            f.held = s.value
        if state.tick % PERIOD == 0:
            box[0] = None
            out = comp.tick(state.tick, samples)
            acc = next((s for s in samples if s.sensor_id == "accel" and s.channel == "z"), None)
            bar = next((s for s in samples if s.sensor_id == "baro"), None)
            bvalid = bar is not None and not bar.saturated and math.isfinite(bar.value)
            avalid = acc is not None and not acc.saturated and math.isfinite(acc.value)
            rows.append((state.tick, state.altitude_agl_m, state.velocity_mps, state.on_ground,
                         acc.value if acc else None, avalid, bar.value if bar else None, bvalid,
                         comp._baro_altitude_m if (bvalid and comp.calibration is not None) else None,
                         box[0], out.estimate.velocity_mps, out.estimate.altitude_agl_m,
                         out.estimate.quality.value, out.flight_state.value))
            events.extend(out.events)
            commands.extend((state.tick, k.value) for k in out.commands)
            pending = out.commands
        if sim.finished or (stop_tick is not None and state.tick >= stop_tick):
            break
        state = sim.step(pending); pending = ()
    return dict(rows=rows, events=[(e.event.value, e.tick, e.source.value, e.quality.value) for e in events],
                commands=commands, truth=sim.truth_events, cal=comp.calibration,
                cal_ticks=av.calibration_duration_ms)


# ------------------------------------------------------------------ derived streams
def phase_of(tick, te):
    if te.liftoff_tick is None or tick < te.liftoff_tick:
        return "PAD"
    if te.burnout_tick is not None and tick < te.burnout_tick:
        return "BOOST"
    if te.apogee_tick is None:
        return "COAST"
    if tick < te.apogee_tick - NEAR_APOGEE_MS:
        return "COAST"
    if tick <= te.apogee_tick + NEAR_APOGEE_MS:
        return "NEAR_APOGEE"
    if te.touchdown_tick is None or tick < te.touchdown_tick:
        return "DESCENT"
    return "LANDED"


def _p2_weights(n):
    xs = [(i - (n - 1) / 2) * BARO_DT / 1000.0 for i in range(n)]
    m2 = sum(x * x for x in xs) / n
    p2 = [x * x - m2 for x in xs]
    den = sum(p * p for p in p2)
    return [2.0 * p / den for p in p2]          # a_fit = sum(w_i * y_i)


_W = {w: _p2_weights(int(round(w * 1000 / BARO_DT))) for w in WINDOWS_S}


def derive(flight):
    """Per valid barometer sample: innovation, dead-reckoning disagreement D, windowed dD_W, plausibility P_W."""
    rows, cal = flight["rows"], flight["cal"]
    out = []                       # (tick, r, D, {W: dD}, {W: P}, h_baro)
    if cal is None:
        return out, []
    h = v = 0.0
    a_held, a_tick = 0.0, None
    acc_series = {}                # tick -> accel-derived acceleration used this tick (None = dead reckoning invalid)
    dr_valid = True
    baro_hist = {}                 # tick -> (h_baro, D)
    cum_a = [0.0]; cum_ticks = []  # prefix sums of acceleration per avionics tick
    tick_index = {}
    started = False
    for row in rows:
        tick, _, _, _, aval, avalid, _, bvalid, hb, r = row[:10]
        if tick < flight["cal_ticks"]:
            continue
        if avalid:
            a_held, a_tick = (aval - cal.accel_bias_mps2) - G0_MPS2, tick
        a_ok = a_tick is not None and tick - a_tick <= 100
        if not a_ok:
            dr_valid = False
        if started:
            h += v * T + 0.5 * a_held * T * T
            v += a_held * T
        started = True
        tick_index[tick] = len(cum_ticks)
        cum_ticks.append(tick)
        cum_a.append(cum_a[-1] + (a_held if a_ok else float("nan")))
        if bvalid and hb is not None:
            D = hb - h if dr_valid else None
            baro_hist[tick] = (hb, D)
            dD, P = {}, {}
            for w in WINDOWS_S:
                n = int(round(w * 1000 / BARO_DT))
                # dD: mean of last 5 D minus mean of the 5 D ending W earlier
                if D is not None:
                    a = [baro_hist.get(tick - k * BARO_DT) for k in range(5)]
                    b = [baro_hist.get(tick - n * BARO_DT - k * BARO_DT) for k in range(5)]
                    if all(x is not None and x[1] is not None for x in a + b):
                        dD[w] = sum(x[1] for x in a) / 5 - sum(x[1] for x in b) / 5
                ys = [baro_hist.get(tick - (n - 1 - i) * BARO_DT) for i in range(n)]
                t_start = tick - (n - 1) * BARO_DT
                if all(y is not None for y in ys) and t_start in tick_index:
                    afit = sum(wi * y[0] for wi, y in zip(_W[w], ys))
                    i0, i1 = tick_index[t_start], tick_index[tick] + 1
                    amean = (cum_a[i1] - cum_a[i0]) / (i1 - i0)
                    if amean == amean:
                        P[w] = afit - amean
            out.append((tick, r, D, dD, P, hb, h if dr_valid else None, v if dr_valid else None))
    return out, None


def run_lengths(rows, te, idx):
    """Longest run of identical consecutive raw readings per phase, and identical-pair counts."""
    best, pairs, same = {}, {}, {}
    prev, run, prev_phase = None, 0, None
    for row in rows:
        if row[idx] is None:
            continue
        val = row[idx]
        ph = phase_of(row[0], te)
        if ph != prev_phase:
            prev, run, prev_phase = None, 0, ph
        if prev is not None:
            pairs[ph] = pairs.get(ph, 0) + 1
            if val == prev:
                same[ph] = same.get(ph, 0) + 1
                run += 1
            else:
                run = 1
        else:
            run = 1
        best[ph] = max(best.get(ph, 0), run)
        prev = val
    return best, pairs, same
