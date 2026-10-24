#!/usr/bin/env python3
"""
Scientific 11-DOF Trajectory Generator for UAV Localization Dataset.
Generates 10 reproducible, C^3 continuous flight profiles (900 seconds each)
within a 200m x 200m arena (altitudes up to 80m AGL).

Outputs 11-DOF setpoints at specified frequency (default: 50 Hz):
[x, y, z, vx, vy, vz, ax, ay, az, yaw, yaw_rate] in NED frame.
"""

import numpy as np
from typing import Dict, Any, Tuple

ARENA_RADIUS = 90.0  # Safe boundary inside 200m x 200m perimeter
DEFAULT_DT = 0.02    # 50 Hz setpoint rate (20 ms interval)
TOTAL_TIME = 900.0   # 15 minutes (seconds)


def get_scenario_metadata() -> Dict[int, Dict[str, Any]]:
    return {
        1: {
            "seed": 1001,
            "name": "3D Lissajous & Multi-Altitude Loops",
            "description": "Smooth, continuous acceleration across varied harmonic frequencies."
        },
        2: {
            "seed": 1002,
            "name": "High-Speed Figure-8s with Altitude Variations",
            "description": "Centripetal acceleration and bank angle excitation."
        },
        3: {
            "seed": 1003,
            "name": "Rapid Velocity Step Responses & Sharp Heading Slaloms",
            "description": "Transient acceleration steps and high angular yaw rate transitions."
        },
        4: {
            "seed": 1004,
            "name": "Spiral Ascents / Descents & Vertical High-G Dives",
            "description": "Wide vertical dynamics exciting barometer and vertical accelerometer."
        },
        5: {
            "seed": 1005,
            "name": "Low-Altitude Ground-Hugging (Multipath Stress)",
            "description": "Operates 3m to 12m AGL near ground clutter, severe ground bounce."
        },
        6: {
            "seed": 1006,
            "name": "Long-Range Perimeter Box Patrol with Speed Variations",
            "description": "Perimeter cruising (2 m/s to 14 m/s) with steady-state cruising segments."
        },
        7: {
            "seed": 1007,
            "name": "High Bank Angle Agile Turns & S-Curves",
            "description": "Severe cross-polarization antenna tilt loss and rapid attitude changes."
        },
        8: {
            "seed": 1008,
            "name": "Periodic Dash-and-Hover with Rapid Braking",
            "description": "High-G deceleration and transient IMU settling."
        },
        9: {
            "seed": 1009,
            "name": "Dense 3D Polyline Waypoint Grid Survey",
            "description": "Lawnmower inspection pattern with frequent 90-degree corners."
        },
        10: {
            "seed": 1010,
            "name": "Aggressive Pseudo-Random 3D Spline Path",
            "description": "Broadband spectral excitation of all IMU axes."
        }
    }


def compute_numerical_derivatives(pos: np.ndarray, t: np.ndarray, yaw: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Computes velocity, acceleration, and yaw rate using central finite differences.
    pos: [N x 3] (NED)
    t: [N]
    yaw: [N]
    """
    dt = t[1] - t[0]
    vel = np.gradient(pos, dt, axis=0)
    acc = np.gradient(vel, dt, axis=0)
    
    # Unwrap yaw before gradient to avoid 2*pi jumps
    yaw_unwrapped = np.unwrap(yaw)
    yaw_rate = np.gradient(yaw_unwrapped, dt)
    
    return vel, acc, yaw_rate


def generate_trajectory(scenario_id: int, total_time: float = TOTAL_TIME, dt: float = DEFAULT_DT) -> Dict[str, np.ndarray]:
    """
    Generates 11-DOF continuous trajectory in NED frame.
    All z coordinates are negative (z_NED = -altitude_AGL).
    """
    meta = get_scenario_metadata().get(scenario_id)
    if not meta:
        raise ValueError(f"Unknown scenario ID: {scenario_id}. Choose 1 to 10.")
    
    np.random.seed(meta["seed"])
    t = np.arange(0.0, total_time + dt, dt)
    N = len(t)
    
    pos = np.zeros((N, 3), dtype=np.float64)
    yaw = np.zeros(N, dtype=np.float64)
    
    # Smooth takeoff ramp in first 10 seconds
    takeoff_mask = t < 10.0
    takeoff_frac = np.clip(t / 10.0, 0.0, 1.0)
    # Smoothstep interpolation: 3x^2 - 2x^3
    smooth_scale = 3.0 * takeoff_frac**2 - 2.0 * takeoff_frac**3

    if scenario_id == 1:
        # 3D Lissajous
        w1, w2, w3 = 2 * np.pi / 90.0, 2 * np.pi / 60.0, 2 * np.pi / 120.0
        x = ARENA_RADIUS * 0.85 * np.sin(w1 * t)
        y = ARENA_RADIUS * 0.85 * np.sin(w2 * t + np.pi / 4.0)
        z = -(25.0 + 20.0 * np.sin(w3 * t))
        yaw = np.arctan2(np.gradient(y), np.gradient(x))

    elif scenario_id == 2:
        # High-Speed Figure-8
        w = 2 * np.pi / 50.0
        x = ARENA_RADIUS * 0.8 * np.sin(w * t)
        y = ARENA_RADIUS * 0.8 * np.sin(2.0 * w * t) / 2.0
        z = -(30.0 + 15.0 * np.cos(w * t * 0.5))
        yaw = np.arctan2(np.gradient(y), np.gradient(x))

    elif scenario_id == 3:
        # Slalom with heading transitions
        freq = 2 * np.pi / 40.0
        x = ARENA_RADIUS * 0.7 * np.sin(freq * t)
        y = ARENA_RADIUS * 0.7 * np.cos(freq * 0.5 * t)
        z = -(20.0 + 10.0 * np.sin(freq * 2.0 * t))
        yaw = np.sin(freq * t) * (np.pi / 2.0)

    elif scenario_id == 4:
        # Spiral ascents and descents
        r_spiral = ARENA_RADIUS * 0.65
        w_spiral = 2 * np.pi / 35.0
        x = r_spiral * np.cos(w_spiral * t)
        y = r_spiral * np.sin(w_spiral * t)
        # Vertical range: 5m to 75m AGL
        z = -(10.0 + 32.5 * (1.0 + np.sin(2 * np.pi * t / 180.0)))
        yaw = np.mod(w_spiral * t + np.pi / 2.0, 2 * np.pi) - np.pi

    elif scenario_id == 5:
        # Low-altitude ground hugging (3m to 12m)
        w = 2 * np.pi / 60.0
        x = ARENA_RADIUS * 0.75 * np.cos(w * t)
        y = ARENA_RADIUS * 0.75 * np.sin(w * 1.5 * t)
        z = -(4.0 + 4.0 * np.sin(w * 3.0 * t))
        yaw = np.arctan2(np.gradient(y), np.gradient(x))

    elif scenario_id == 6:
        # Perimeter box patrol
        box_side = ARENA_RADIUS * 1.5
        period = 120.0
        phase = np.mod(t, period) / period
        
        # Square corners
        x_box = np.piecewise(phase, 
            [phase < 0.25, (phase >= 0.25) & (phase < 0.5), (phase >= 0.5) & (phase < 0.75), phase >= 0.75],
            [lambda p: -box_side/2 + 4*box_side*p,
             box_side/2,
             lambda p: box_side/2 - 4*box_side*(p - 0.5),
             -box_side/2])
        y_box = np.piecewise(phase, 
            [phase < 0.25, (phase >= 0.25) & (phase < 0.5), (phase >= 0.5) & (phase < 0.75), phase >= 0.75],
            [-box_side/2,
             lambda p: -box_side/2 + 4*box_side*(p - 0.25),
             box_side/2,
             lambda p: box_side/2 - 4*box_side*(p - 0.75)])
        
        # Smooth corner filter via Gaussian convolution
        from scipy.ndimage import gaussian_filter1d
        x = gaussian_filter1d(x_box, sigma=int(3.0 / dt), mode="wrap")
        y = gaussian_filter1d(y_box, sigma=int(3.0 / dt), mode="wrap")
        z = -(35.0 + 10.0 * np.sin(2 * np.pi * t / 200.0))
        yaw = np.arctan2(np.gradient(y), np.gradient(x))

    elif scenario_id == 7:
        # High bank S-curves
        w = 2 * np.pi / 30.0
        x = ARENA_RADIUS * 0.8 * np.sin(w * t)
        y = ARENA_RADIUS * 0.5 * np.sin(2 * w * t)
        z = -(30.0 + 8.0 * np.cos(w * t))
        yaw = np.arctan2(np.gradient(y), np.gradient(x))

    elif scenario_id == 8:
        # Dash-and-hover
        period = 45.0
        phase = np.mod(t, period)
        dash_mask = phase < 20.0
        angle = np.floor(t / period) * (np.pi / 3.0)
        
        r_step = np.where(dash_mask, ARENA_RADIUS * 0.75 * np.sin(np.pi * phase / 20.0), 0.0)
        x = r_step * np.cos(angle)
        y = r_step * np.sin(angle)
        z = -25.0 * np.ones_like(t)
        yaw = angle

    elif scenario_id == 9:
        # Lawnmower 3D grid
        grid_w = ARENA_RADIUS * 1.4
        grid_lanes = 6
        lane_len = ARENA_RADIUS * 1.5
        period = 150.0
        u = np.mod(t, period) / period
        
        x_raw = (2.0 * np.abs(2.0 * (u * grid_lanes - np.floor(u * grid_lanes + 0.5))) - 1.0) * (lane_len / 2.0)
        y_raw = (u - 0.5) * grid_w
        from scipy.ndimage import gaussian_filter1d
        x = gaussian_filter1d(x_raw, sigma=int(2.5 / dt), mode="wrap")
        y = gaussian_filter1d(y_raw, sigma=int(2.5 / dt), mode="wrap")
        z = -(20.0 + 5.0 * np.sin(2 * np.pi * t / 50.0))
        yaw = np.arctan2(np.gradient(y), np.gradient(x))

    else: # scenario 10
        # Broadband pseudo-random spline
        freqs = np.array([0.015, 0.035, 0.08, 0.14])
        phases_x = np.random.uniform(0, 2*np.pi, len(freqs))
        phases_y = np.random.uniform(0, 2*np.pi, len(freqs))
        phases_z = np.random.uniform(0, 2*np.pi, len(freqs))
        
        x = sum(ARENA_RADIUS * (0.8 / len(freqs)) * np.sin(2 * np.pi * f * t + px) for f, px in zip(freqs, phases_x))
        y = sum(ARENA_RADIUS * (0.8 / len(freqs)) * np.cos(2 * np.pi * f * t + py) for f, py in zip(freqs, phases_y))
        z = -(25.0 + sum(12.0 / len(freqs) * np.sin(2 * np.pi * f * t + pz) for f, pz in zip(freqs, phases_z)))
        yaw = np.arctan2(np.gradient(y), np.gradient(x))

    # Apply smooth takeoff scaling to positions
    pos[:, 0] = x * smooth_scale
    pos[:, 1] = y * smooth_scale
    pos[:, 2] = z * smooth_scale
    yaw = yaw * smooth_scale

    vel, acc, yaw_rate = compute_numerical_derivatives(pos, t, yaw)

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
    import argparse
    parser = argparse.ArgumentParser(description="Generate 11-DOF UAV trajectory setpoints.")
    parser.add_argument("--scenario", type=int, default=1, help="Scenario ID (1-10)")
    parser.add_argument("--duration", type=float, default=TOTAL_TIME, help="Duration in seconds")
    parser.add_argument("--dt", type=float, default=DEFAULT_DT, help="Time step in seconds (default: 0.02)")
    args = parser.parse_args()

    traj = generate_trajectory(args.scenario, total_time=args.duration, dt=args.dt)
    print(f"Generated Scenario {args.scenario}: {traj['metadata']['name']}")
    print(f"Total points: {len(traj['time'])} @ {1.0/args.dt:.1f} Hz ({traj['time'][-1]:.1f} s)")
    print(f"Position bounds (NED):")
    print(f"  North (X): [{traj['pos_ned'][:, 0].min():.2f}, {traj['pos_ned'][:, 0].max():.2f}] m")
    print(f"  East  (Y): [{traj['pos_ned'][:, 1].min():.2f}, {traj['pos_ned'][:, 1].max():.2f}] m")
    print(f"  Down  (Z): [{traj['pos_ned'][:, 2].min():.2f}, {traj['pos_ned'][:, 2].max():.2f}] m (AGL: {-traj['pos_ned'][:, 2].max():.2f} to {-traj['pos_ned'][:, 2].min():.2f} m)")
    print(f"Max velocity: {np.linalg.norm(traj['vel_ned'], axis=1).max():.2f} m/s")
    print(f"Max acceleration: {np.linalg.norm(traj['acc_ned'], axis=1).max():.2f} m/s^2")
