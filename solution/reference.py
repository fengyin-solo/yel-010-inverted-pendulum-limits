#!/usr/bin/env python3
"""Saturated nonlinear pendulum simulation and recovery-envelope search."""

from __future__ import annotations

import json
import math
import os
import sys
import tempfile
from pathlib import Path


TOP = {
    "controller_id",
    "mass_kg",
    "length_m",
    "gravity_m_s2",
    "damping_s_inv",
    "gains",
    "torque_limit_nm",
    "simulation",
}
SIM = {
    "dt_s",
    "duration_s",
    "initial_angle_rad",
    "initial_rate_rad_s",
    "recovery_low_rad",
    "recovery_high_rad",
    "settle_angle_rad",
    "settle_rate_rad_s",
    "search_tolerance_rad",
}


class Refusal(Exception):
    def __init__(self, kind):
        self.kind = kind


def fail(kind):
    sys.stderr.write(f"运行拒收: {kind}\n")
    raise SystemExit(1)


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def pairs_hook(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise Refusal("BAD_INPUT")
        result[key] = value
    return result


def validate(payload):
    if not isinstance(payload, dict) or set(payload) != TOP:
        raise Refusal("BAD_INPUT")
    identifier = payload["controller_id"]
    if (
        not isinstance(identifier, str)
        or not 2 <= len(identifier) <= 48
        or identifier != identifier.strip()
        or any(ord(char) < 32 or ord(char) == 127 for char in identifier)
    ):
        raise Refusal("BAD_INPUT")
    checks = [
        ("mass_kg", 0.1, 100.0),
        ("length_m", 0.1, 10.0),
        ("gravity_m_s2", 1.0, 30.0),
        ("damping_s_inv", 0.0, 20.0),
        ("torque_limit_nm", 0.001, 10000.0),
    ]
    for key, low, high in checks:
        value = payload[key]
        if not finite(value) or not low <= value <= high:
            raise Refusal("BAD_INPUT")
    gains = payload["gains"]
    if not isinstance(gains, list) or len(gains) != 2 or any(not finite(value) for value in gains):
        raise Refusal("BAD_INPUT")
    simulation = payload["simulation"]
    if not isinstance(simulation, dict) or set(simulation) != SIM:
        raise Refusal("BAD_INPUT")
    if any(not finite(value) for value in simulation.values()):
        raise Refusal("BAD_INPUT")
    if not 0.0001 <= simulation["dt_s"] <= 0.05 or not 0.1 <= simulation["duration_s"] <= 60:
        raise Refusal("BAD_INPUT")
    angle_keys = (
        "initial_angle_rad",
        "initial_rate_rad_s",
        "recovery_low_rad",
        "recovery_high_rad",
    )
    if any(abs(simulation[key]) > math.pi for key in angle_keys):
        raise Refusal("BAD_INPUT")
    if simulation["recovery_low_rad"] >= simulation["recovery_high_rad"]:
        raise Refusal("BAD_INPUT")
    for key in ("settle_angle_rad", "settle_rate_rad_s", "search_tolerance_rad"):
        if simulation[key] <= 0:
            raise Refusal("BAD_INPUT")
    if simulation["search_tolerance_rad"] > 1e-4:
        raise Refusal("BAD_INPUT")
    return payload


def dynamics(state, payload):
    angle, rate = state
    k_angle, k_rate = payload["gains"]
    raw = -(k_angle * angle + k_rate * rate)
    torque = max(-payload["torque_limit_nm"], min(payload["torque_limit_nm"], raw))
    acceleration = (
        payload["gravity_m_s2"] / payload["length_m"] * math.sin(angle)
        - payload["damping_s_inv"] * rate
        + torque / (payload["mass_kg"] * payload["length_m"] ** 2)
    )
    return rate, acceleration, abs(torque)


def rk4_step(state, payload, dt):
    a1 = dynamics(state, payload)
    a2 = dynamics((state[0] + dt * a1[0] / 2, state[1] + dt * a1[1] / 2), payload)
    a3 = dynamics((state[0] + dt * a2[0] / 2, state[1] + dt * a2[1] / 2), payload)
    a4 = dynamics((state[0] + dt * a3[0], state[1] + dt * a3[1]), payload)
    next_state = (
        state[0] + dt * (a1[0] + 2 * a2[0] + 2 * a3[0] + a4[0]) / 6,
        state[1] + dt * (a1[1] + 2 * a2[1] + 2 * a3[1] + a4[1]) / 6,
    )
    peak = max(a1[2], a2[2], a3[2], a4[2])
    return next_state, peak


def simulate(payload, initial_angle, initial_rate):
    simulation = payload["simulation"]
    remaining = simulation["duration_s"]
    state = (initial_angle, initial_rate)
    peak = 0.0
    while remaining > 1e-15:
        step = min(simulation["dt_s"], remaining)
        state, step_peak = rk4_step(state, payload, step)
        peak = max(peak, step_peak)
        remaining -= step
    return state, peak


def recovered(state, simulation):
    return (
        abs(state[0]) <= simulation["settle_angle_rad"]
        and abs(state[1]) <= simulation["settle_rate_rad_s"]
    )


def evaluate(payload):
    simulation = payload["simulation"]
    state, peak = simulate(
        payload,
        simulation["initial_angle_rad"],
        simulation["initial_rate_rad_s"],
    )
    low = simulation["recovery_low_rad"]
    high = simulation["recovery_high_rad"]
    low_state, _ = simulate(payload, low, 0.0)
    if not recovered(low_state, simulation):
        raise Refusal("NO_RECOVERY")
    for _ in range(48):
        middle = (low + high) / 2.0
        middle_state, _ = simulate(payload, middle, 0.0)
        if recovered(middle_state, simulation):
            low = middle
        else:
            high = middle
    return {
        "controller_id": payload["controller_id"],
        "status": "evaluated",
        "final_angle_rad": state[0],
        "final_rate_rad_s": state[1],
        "peak_torque_nm": peak,
        "recovery_angle_rad": low,
        "recovered": recovered(state, simulation),
    }


def publish(target, payload):
    if not target.parent.is_dir() or target.is_dir() or not os.access(target.parent, os.W_OK):
        raise Refusal("IO_FAILURE")
    old = target.read_bytes() if target.exists() else None
    stage = None
    try:
        descriptor, name = tempfile.mkstemp(prefix=".pendulum-", dir=target.parent)
        stage = Path(name)
        with os.fdopen(descriptor, "wb") as handle:
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(stage, target)
        stage = None
    except OSError:
        if old is None:
            target.unlink(missing_ok=True)
        else:
            target.write_bytes(old)
        raise Refusal("IO_FAILURE")
    finally:
        if stage is not None:
            stage.unlink(missing_ok=True)


def main(argv):
    if len(argv) != 3:
        fail("BAD_INPUT")
    source, target = Path(argv[1]), Path(argv[2])
    try:
        payload = json.loads(source.read_text(encoding="utf-8"), object_pairs_hook=pairs_hook)
        result = evaluate(validate(payload))
        encoded = (json.dumps(result, ensure_ascii=False, separators=(",", ":")) + "\n").encode()
        publish(target, encoded)
        return 0
    except Refusal as exc:
        fail(exc.kind)
    except (OSError, UnicodeError, ValueError):
        fail("BAD_INPUT")


if __name__ == "__main__":
    sys.exit(main(sys.argv))
