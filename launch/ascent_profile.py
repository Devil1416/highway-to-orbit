"""Simple gravity-turn ascent profile for launch simulation."""
import math
from dataclasses import dataclass

@dataclass
class AscentState:
    altitude: float    # m
    velocity: float    # m/s
    flight_angle: float  # radians from vertical

def gravity_turn_step(state: AscentState, thrust: float, mass: float,
                      dt: float, g: float = 9.80665) -> AscentState:
    """Advance ascent state by dt seconds using gravity-turn steering."""
    drag = 0.0  # simplified: no atmosphere model here
    a_thrust = thrust / mass
    a_gravity_parallel = g * math.cos(state.flight_angle)
    a_gravity_perp    = g * math.sin(state.flight_angle)

    v_dot = a_thrust - a_gravity_parallel - drag
    angle_dot = -a_gravity_perp / max(state.velocity, 1.0)  # avoid div/0

    new_v     = state.velocity + v_dot * dt
    new_angle = state.flight_angle + angle_dot * dt
    new_alt   = state.altitude + state.velocity * math.cos(state.flight_angle) * dt
    return AscentState(new_alt, new_v, new_angle)
