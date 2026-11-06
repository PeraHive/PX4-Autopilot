#!/usr/bin/env python3
"""
11-DOF Trajectory Generator for 10 distinct flight scenarios.
Outputs continuous [x, y, z, vx, vy, vz, ax, ay, az, yaw, yaw_rate] in NED frame.
"""

import numpy as np
from typing import Dict, Any, Tuple
from scipy.ndimage import gaussian_filter1d

ARENA_RADIUS = 90.0  # 200m x 200m arena perimeter
TOTAL_TIME = 900.0   # 15 minutes


def get_scenario_metadata() -> Dict[int, Dict[str, Any]]:
    return {
        1: {"seed": 1001, "name": "3D Lissajous & Multi-Altitude Loops", "description": "Continuous smooth harmonic acceleration."},
        2: {"seed": 1002, "name": "High-Speed Figure-8s with Altitude Variations", "description": "Centripetal acceleration and bank angle dynamics."},
        3: {"seed": 1003, "name": "Rapid Step Responses & Heading Slaloms", "description": "Transient acceleration steps and high yaw rate transitions."},
        4: {"seed": 1004, "name": "Spiral Ascents/Descents & Dives", "description": "High vertical rate dynamic changes."},
        5: {"seed": 1005, "name": "Low-Altitude Ground-Hugging (Multipath Stress)", "description": "Near-ground 3-12m operations."},
        6: {"seed": 1006, "name": "Perimeter Box Patrol", "description": "2-14 m/s steady-state cruising."},
        7: {"seed": 1007, "name": "Agile High-Bank Turns", "description": "Severe cross-polarization tilt loss."},
        8: {"seed": 1008, "name": "Periodic Dash-and-Hover", "description": "Rapid braking and transient IMU settling."},
        9: {"seed": 1009, "name": "Dense 3D Lawnmower Grid Survey", "description": "Orthogonal survey patterns with 90-deg turns."},
        10: {"seed": 1010, "name": "Broadband Pseudo-Random Spline", "description": "Wideband spectral excitation of all IMU axes."}
    }


def compute_derivatives(pos: np.ndarray, t: np.ndarray, yaw: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    dt = t[1] - t[0]
    vel = np.gradient(pos, dt, axis=0)
    acc = np.gradient(vel, dt, axis=0)
    yaw_rate = np.gradient(np.unwrap(yaw), dt)
    return vel, acc, yaw_rate


def generate_trajectory(scenario_id: int, total_time: float = TOTAL_TIME, dt: float = 0.05) -> Dict[str, np.ndarray]:
    meta = get_scenario_metadata().get(scenario_id, get_scenario_metadata()[1])
    np.random.seed(meta["seed"])
    t = np.arange(0.0, total_time + dt, dt)
    N = len(t)
    pos = np.zeros((N, 3), dtype=np.float64)

    # Smooth takeoff
    smooth = np.clip(t / 8.0, 0.0, 1.0)
    smooth = 3.0 * smooth**2 - 2.0 * smooth**3

    if scenario_id == 1:
        w1, w2, w3 = 2*np.pi/90.0, 2*np.pi/60.0, 2*np.pi/120.0
        x = ARENA_RADIUS * 0.85 * np.sin(w1 * t)
        y = ARENA_RADIUS * 0.85 * np.sin(w2 * t + np.pi/4.0)
        z = -(25.0 + 20.0 * np.sin(w3 * t))
    elif scenario_id == 2:
        w = 2*np.pi/50.0
        x = ARENA_RADIUS * 0.8 * np.sin(w * t)
        y = ARENA_RADIUS * 0.8 * np.sin(2.0 * w * t) / 2.0
        z = -(30.0 + 15.0 * np.cos(w * t * 0.5))
    elif scenario_id == 3:
        freq = 2*np.pi/40.0
        x = ARENA_RADIUS * 0.7 * np.sin(freq * t)
        y = ARENA_RADIUS * 0.7 * np.cos(freq * 0.5 * t)
        z = -(20.0 + 10.0 * np.sin(freq * 2.0 * t))
    elif scenario_id == 4:
        w = 2*np.pi/35.0
        x = ARENA_RADIUS * 0.65 * np.cos(w * t)
        y = ARENA_RADIUS * 0.65 * np.sin(w * t)
        z = -(10.0 + 30.0 * (1.0 + np.sin(2 * np.pi * t / 180.0)))
    elif scenario_id == 5:
        w = 2*np.pi/60.0
        x = ARENA_RADIUS * 0.75 * np.cos(w * t)
        y = ARENA_RADIUS * 0.75 * np.sin(w * 1.5 * t)
        z = -(5.0 + 3.0 * np.sin(w * 3.0 * t))
    elif scenario_id == 6:
        side = ARENA_RADIUS * 1.4
        period = 120.0
        u = np.mod(t, period) / period
        x_raw = np.piecewise(u, [u < 0.25, (u >= 0.25) & (u < 0.5), (u >= 0.5) & (u < 0.75), u >= 0.75],
                             [lambda p: -side/2 + 4*side*p, side/2, lambda p: side/2 - 4*side*(p-0.5), -side/2])
        y_raw = np.piecewise(u, [u < 0.25, (u >= 0.25) & (u < 0.5), (u >= 0.5) & (u < 0.75), u >= 0.75],
                             [-side/2, lambda p: -side/2 + 4*side*(p-0.25), side/2, lambda p: side/2 - 4*side*(p-0.75)])
        x = gaussian_filter1d(x_raw, sigma=int(2.5/dt), mode="wrap")
        y = gaussian_filter1d(y_raw, sigma=int(2.5/dt), mode="wrap")
        z = -(30.0 + 10.0 * np.sin(2 * np.pi * t / 200.0))
    elif scenario_id == 7:
        w = 2*np.pi/30.0
        x = ARENA_RADIUS * 0.8 * np.sin(w * t)
        y = ARENA_RADIUS * 0.5 * np.sin(2 * w * t)
        z = -(30.0 + 8.0 * np.cos(w * t))
    elif scenario_id == 8:
        period = 45.0
        phase = np.mod(t, period)
        dash = phase < 20.0
        angle = np.floor(t / period) * (np.pi / 3.0)
        r = np.where(dash, ARENA_RADIUS * 0.75 * np.sin(np.pi * phase / 20.0), 0.0)
        x = r * np.cos(angle)
        y = r * np.sin(angle)
        z = -25.0 * np.ones_like(t)
    elif scenario_id == 9:
        w_grid = ARENA_RADIUS * 1.4
        lanes = 6
        lane_len = ARENA_RADIUS * 1.5
        period = 150.0
        u = np.mod(t, period) / period
        x_raw = (2.0 * np.abs(2.0 * (u * lanes - np.floor(u * lanes + 0.5))) - 1.0) * (lane_len / 2.0)
        y_raw = (u - 0.5) * w_grid
        x = gaussian_filter1d(x_raw, sigma=int(2.0/dt), mode="wrap")
        y = gaussian_filter1d(y_raw, sigma=int(2.0/dt), mode="wrap")
        z = -(20.0 + 5.0 * np.sin(2 * np.pi * t / 50.0))
    else:
        freqs = [0.015, 0.035, 0.08, 0.14]
        px = np.random.uniform(0, 2*np.pi, len(freqs))
        py = np.random.uniform(0, 2*np.pi, len(freqs))
        pz = np.random.uniform(0, 2*np.pi, len(freqs))
        x = sum(ARENA_RADIUS * (0.8/len(freqs)) * np.sin(2*np.pi*f*t + p) for f, p in zip(freqs, px))
        y = sum(ARENA_RADIUS * (0.8/len(freqs)) * np.cos(2*np.pi*f*t + p) for f, p in zip(freqs, py))
        z = -(25.0 + sum(12.0/len(freqs) * np.sin(2*np.pi*f*t + p) for f, p in zip(freqs, pz)))

    pos[:, 0] = x * smooth
    pos[:, 1] = y * smooth
    pos[:, 2] = z * smooth
    yaw = np.arctan2(np.gradient(pos[:, 1]), np.gradient(pos[:, 0])) * smooth

    vel, acc, yaw_rate = compute_derivatives(pos, t, yaw)

    return {
        "time": t,
        "pos_ned": pos,
        "vel_ned": vel,
        "acc_ned": acc,
        "yaw_rad": yaw,
        "yaw_rate_rad_s": yaw_rate,
        "scenario_id": scenario_id,
        "metadata": meta
    }


if __name__ == "__main__":
    t = generate_trajectory(1, total_time=10.0)
    print("Trajectory generated:", t["pos_ned"].shape)
