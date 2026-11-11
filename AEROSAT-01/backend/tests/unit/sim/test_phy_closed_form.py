"""PHY-06 to PHY-10 and PHY-18 (TEST_PLAN section 4): integration against closed-form oracles.

Every expected value comes from a closed-form expression or an invariant written out in this
file. None comes from the code under test. Tolerances are the ones fixed in TEST_PLAN.
"""
import math

import pytest

from aerosat.core.constants import G0_MPS2, R0_M
from aerosat.sim.atmosphere import ConstantDensityAtmosphere, VacuumAtmosphere
from aerosat.sim.dynamics import FlightModel
from aerosat.sim.gravity import constant_gravity_mps2, inverse_square_gravity_mps2
from aerosat.sim.integrators import euler_step, rk4_step
from aerosat.sim.simulator import Simulator, SimulatorConfig
from aerosat.sim.vehicle import Vehicle

G = 9.80665


def test_phy_06_vacuum_constant_gravity_parabola(unpowered_vehicle: Vehicle) -> None:
    """v0 = 50 m/s upward for 10 s. Oracle: h = v0*t - g*t^2/2, v = v0 - g*t."""
    sim = Simulator(SimulatorConfig(
        vehicle=unpowered_vehicle, atmosphere=VacuumAtmosphere(), gravity_mps2=constant_gravity_mps2,
        initial_velocity_mps=50.0,
    ))
    worst_h = worst_v = 0.0
    for _ in range(10_000):
        state = sim.step()
        t = state.t_sim_s
        worst_h = max(worst_h, abs(state.altitude_agl_m - (50.0 * t - 0.5 * G * t * t)))
        worst_v = max(worst_v, abs(state.velocity_mps - (50.0 - G * t)))
    assert state.t_sim_s == 10.0
    assert not state.on_ground
    assert worst_h <= 1e-9
    assert worst_v <= 1e-9


def test_phy_07_upward_coast_with_quadratic_drag(unpowered_vehicle: Vehicle, terminal_velocity_mps: float) -> None:
    """v0 = 200 m/s upward, constant density and gravity. Oracle: TEST_PLAN section 4 closed form."""
    v_t = terminal_velocity_mps
    phi0 = math.atan(200.0 / v_t)
    t_apogee = v_t / G * phi0
    h_apogee = v_t * v_t / (2 * G) * math.log(1 + 200.0**2 / v_t**2)
    assert t_apogee == pytest.approx(16.5314, abs=5e-5)   # the values printed in TEST_PLAN
    assert h_apogee == pytest.approx(1483.659, abs=5e-4)

    sim = Simulator(SimulatorConfig(
        vehicle=unpowered_vehicle, atmosphere=ConstantDensityAtmosphere(1.225),
        gravity_mps2=constant_gravity_mps2, initial_velocity_mps=200.0,
    ))
    at_8_s = None
    while sim.state.velocity_mps > -1.0:
        state = sim.step()
        if state.tick == 8_000:
            at_8_s = state
    assert at_8_s is not None
    angle = phi0 - G * 8.0 / v_t
    assert abs(at_8_s.velocity_mps - v_t * math.tan(angle)) <= 1e-6
    assert abs(at_8_s.altitude_agl_m - v_t * v_t / G * math.log(math.cos(angle) / math.cos(phi0))) <= 1e-6

    events = sim.truth_events
    assert abs(events.apogee_tick / 1000 - t_apogee) <= 1e-3
    assert abs(events.apogee_altitude_agl_m - h_apogee) <= 1e-3


def test_phy_08_fall_from_rest_10_s(unpowered_vehicle: Vehicle, terminal_velocity_mps: float) -> None:
    """Released at rest 1000 m above the pad. Oracle: v = -v_t*tanh(g*t/v_t), h = -(v_t^2/g)*ln cosh(g*t/v_t)."""
    v_t = terminal_velocity_mps
    sim = Simulator(SimulatorConfig(
        vehicle=unpowered_vehicle, atmosphere=ConstantDensityAtmosphere(1.225),
        gravity_mps2=constant_gravity_mps2, initial_altitude_agl_m=1000.0,
    ))
    for _ in range(10_000):
        state = sim.step()
    x = G * 10.0 / v_t
    assert abs(state.velocity_mps - (-v_t * math.tanh(x))) <= 1e-6
    assert abs((state.altitude_agl_m - 1000.0) - (-(v_t * v_t / G) * math.log(math.cosh(x)))) <= 1e-6
    assert not state.on_ground


def _errors_at_8_s(step, dt: float, vehicle: Vehicle, v_t: float) -> tuple[float, float]:
    model = FlightModel(vehicle, t_ignition_s=0.0, atmosphere=ConstantDensityAtmosphere(1.225),
                        gravity_mps2=constant_gravity_mps2, pad_altitude_msl_m=0.0)
    f = model.derivative(vehicle.cd_area_m2)
    y = (0.0, 200.0)
    steps = round(8.0 / dt)
    for i in range(steps):
        y = step(f, i * dt, y, dt)
    phi0 = math.atan(200.0 / v_t)
    angle = phi0 - G * 8.0 / v_t
    return (
        abs(y[0] - v_t * v_t / G * math.log(math.cos(angle) / math.cos(phi0))),
        abs(y[1] - v_t * math.tan(angle)),
    )


@pytest.mark.parametrize(("step", "low", "high"), [(euler_step, 0.8, 1.2), (rk4_step, 3.5, 4.5)],
                         ids=["euler", "rk4"])
def test_phy_09_convergence_order(step, low: float, high: float, unpowered_vehicle: Vehicle,
                                  terminal_velocity_mps: float) -> None:
    """PHY-07 case with coarse steps 0.4, 0.2, 0.1 s. Theory: Euler order 1, RK4 order 4."""
    errors = {dt: _errors_at_8_s(step, dt, unpowered_vehicle, terminal_velocity_mps) for dt in (0.4, 0.2, 0.1)}
    for index, quantity in enumerate(("altitude", "velocity")):
        for coarse, fine in ((0.4, 0.2), (0.2, 0.1)):
            order = math.log2(errors[coarse][index] / errors[fine][index])
            assert low <= order <= high, f"{quantity} order {order:.3f} between dt={coarse} and dt={fine}"


def test_phy_10_energy_conservation_inverse_square_gravity(unpowered_vehicle: Vehicle) -> None:
    """No thrust, no drag, inverse-square gravity, 60 s. Invariant: v^2/2 - g0*r0^2/(r0+h)."""
    sim = Simulator(SimulatorConfig(
        vehicle=unpowered_vehicle, atmosphere=VacuumAtmosphere(), gravity_mps2=inverse_square_gravity_mps2,
        initial_velocity_mps=400.0,
    ))

    def energy(state) -> float:
        return state.velocity_mps**2 / 2 - G0_MPS2 * R0_M**2 / (R0_M + state.altitude_msl_m)

    e0 = energy(sim.state)
    worst = 0.0
    for _ in range(60_000):
        state = sim.step()
        worst = max(worst, abs(energy(state) - e0) / abs(e0))
    assert not state.on_ground
    assert worst <= 1e-9


def test_phy_18_integrator_stress_300_s(terminal_velocity_mps: float) -> None:
    """Numerical integrator stress test. NOT a flight scenario.

    RK4 applied directly to dv/dt = -g - (g/v_t^2)*v*|v| from rest for 300 s at dt = 1 ms.
    Velocity is the only state, so |v| <= v_t throughout. No atmosphere model, no simulator,
    no range check. Oracle: the closed-form limit v_t.
    """
    v_t = terminal_velocity_mps
    k = G / (v_t * v_t)

    def f(t: float, y: tuple[float, ...]) -> tuple[float, ...]:
        v = y[0]
        return (-G - k * v * abs(v),)

    y = (0.0,)
    dt = 0.001
    for i in range(300_000):
        y = rk4_step(f, i * dt, y, dt)
        assert math.isfinite(y[0])
        assert abs(y[0]) <= v_t * (1 + 1e-12)
    assert abs(abs(y[0]) - v_t) / v_t <= 1e-6
