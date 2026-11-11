"""RK4, Euler.

ARCHITECTURE section 4.9. Fixed-step integrators for y' = f(t, y), where y is a tuple of floats.
They know nothing about rockets, so they can be tested on any equation.
"""
from collections.abc import Callable

State = tuple[float, ...]
Derivative = Callable[[float, State], State]
Integrator = Callable[[Derivative, float, State, float], State]


def euler_step(f: Derivative, t: float, y: State, dt: float) -> State:
    """Explicit Euler: y + dt*k1. First order. Selectable for teaching comparisons."""
    k1 = f(t, y)
    return tuple(yi + dt * ki for yi, ki in zip(y, k1))


def rk4_step(f: Derivative, t: float, y: State, dt: float) -> State:
    """Classical fourth-order Runge-Kutta."""
    half = dt / 2
    k1 = f(t, y)
    k2 = f(t + half, tuple(yi + half * ki for yi, ki in zip(y, k1)))
    k3 = f(t + half, tuple(yi + half * ki for yi, ki in zip(y, k2)))
    k4 = f(t + dt, tuple(yi + dt * ki for yi, ki in zip(y, k3)))
    sixth = dt / 6
    return tuple(
        yi + sixth * (a + 2 * b + 2 * c + d) for yi, a, b, c, d in zip(y, k1, k2, k3, k4)
    )


INTEGRATORS: dict[str, Integrator] = {"rk4": rk4_step, "euler": euler_step}
