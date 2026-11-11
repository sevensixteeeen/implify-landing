"""g0, r0, T0, p0, L, R, gamma: the only place they appear.

Standard published constants (ARCHITECTURE section 4.2). They are not fictional parameters.
"""

G0_MPS2 = 9.80665
"""Standard gravity, m/s^2."""

R0_M = 6_356_766.0
"""Reference Earth radius used by the standard atmosphere, m."""

T0_K = 288.15
"""Sea-level standard temperature, K."""

P0_PA = 101_325.0
"""Sea-level standard pressure, Pa."""

LAPSE_RATE = 0.0065
"""Temperature lapse rate L for 0-11 km, in K/m (no unit suffix exists for K/m)."""

R_AIR = 287.05287
"""Specific gas constant of dry air R, in J/(kg*K) (no unit suffix exists for J/(kg*K))."""

GAMMA = 1.4
"""Ratio of specific heats of air. Dimensionless."""
