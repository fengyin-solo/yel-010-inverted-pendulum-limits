#!/usr/bin/env python3
"""Legacy linear Euler simulator."""

import json
import sys
from pathlib import Path


def main(argv):
    if len(argv) != 3:
        return 1
    source, target = Path(argv[1]), Path(argv[2])
    payload = json.loads(source.read_text(encoding="utf-8"))
    config = payload["simulation"]
    angle = config["initial_angle_rad"]
    rate = config["initial_rate_rad_s"]
    k_angle, k_rate = payload["gains"]
    dt = config["dt_s"]
    steps = int(config["duration_s"] / dt)
    peak = 0.0
    for _ in range(steps):
        torque = -(k_angle * angle + k_rate * rate)
        peak = max(peak, abs(torque))
        acceleration = payload["gravity_m_s2"] / payload["length_m"] * angle - payload["damping_s_inv"] * rate
        angle += rate * dt
        rate += acceleration * dt
    result = {
        "controller_id": payload["controller_id"],
        "status": "evaluated",
        "final_angle_rad": angle,
        "final_rate_rad_s": rate,
        "peak_torque_nm": peak,
        "recovery_angle_rad": config["recovery_high_rad"],
        "recovered": abs(angle) <= config["settle_angle_rad"],
    }
    target.write_text(json.dumps(result, separators=(",", ":")) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
