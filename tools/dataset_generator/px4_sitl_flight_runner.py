#!/usr/bin/env python3
"""
Automated PX4 SITL Mission Runner.
Connects to PX4 SITL via MAVSDK-Python, arms the vehicle, and flies 11-DOF
trajectories over UDP 14540 while PX4's internal logger records full uORB flight logs.
"""

import asyncio
import os
import sys
import subprocess
import time
import numpy as np
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw, VelocityNedYaw)

script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(script_dir, "../.."))
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from tools.dataset_generator.trajectory_generator import generate_trajectory


async def run_sitl_flight(
    scenario_id: int,
    duration_s: float = 60.0,
    px4_binary: str = "build/px4_sitl_default/bin/px4",
    px4_rootfs: str = "build/px4_sitl_default/etc",
    mavsdk_port: int = 14540
) -> str:
    """
    Spawns PX4 SITL with SIH physics, executes the trajectory via MAVSDK,
    disarms, and returns the path to the newly generated .ulg log file.
    """
    print(f"\n=======================================================")
    print(f"Launching Native PX4 SITL for Scenario {scenario_id:02d}")
    print(f"Physics Engine: SIH (250 Hz rigid-body lockstep)")
    print(f"Flight Duration: {duration_s:.1f} s")
    print(f"=======================================================")

    # Ensure no stale PX4 processes
    subprocess.run(["killall", "-9", "px4"], stderr=subprocess.DEVNULL)
    await asyncio.sleep(1.0)

    # Clean existing logs directory to locate new log easily
    log_dir = os.path.abspath("fs/log")
    os.makedirs(log_dir, exist_ok=True)
    before_logs = set()
    for root, _, files in os.walk(log_dir):
        for f in files:
            if f.endswith(".ulg"):
                before_logs.add(os.path.join(root, f))

    # Environment settings for PX4 SITL
    env = os.environ.copy()
    env["PX4_SYS_AUTOSTART"] = "10040"  # SIH QuadX
    env["PX4_PARAM_SDLOG_PROFILE"] = "31"  # Bits 0-4 (EKF2 replay, high-rate raw IMU/baro, sysid, high-rate actuators)
    env["PX4_PARAM_SDLOG_MODE"] = "0"     # Armed until disarm
    env["PX4_PARAM_SIM_BAT_DRAIN"] = "0"  # Prevent battery depletion
    env["PX4_PARAM_COM_LOW_BAT_ACT"] = "0"

    px4_proc = subprocess.Popen(
        [px4_binary, px4_rootfs, "-d", "-s", "etc/init.d-posix/rcS"],
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )

    print(f"PX4 SITL started (PID: {px4_proc.pid}), connecting MAVSDK to udp://:{mavsdk_port}...")
    drone = System()
    await drone.connect(system_address=f"udp://:{mavsdk_port}")

    print("Waiting for drone to initialize...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-> Connected to PX4 SITL (System ID: {state})")
            break

    print("Waiting for global position & home position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-> Position and Home fix acquired!")
            break
        await asyncio.sleep(0.5)

    # Arm vehicle
    print("Arming vehicle...")
    try:
        await drone.action.arm()
        print("-> Armed successfully!")
    except Exception as e:
        print(f"Arming failed: {e}")

    # Set initial setpoint before starting offboard
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -5.0, 0.0))
    print("Starting offboard mode...")
    try:
        await drone.offboard.start()
        print("-> Offboard mode active!")
    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: {error._result.result}")
        px4_proc.kill()
        return ""

    # Stream 11-DOF trajectory setpoints at 20 Hz
    dt = 0.05
    traj = generate_trajectory(scenario_id, total_time=duration_s, dt=dt)
    n_points = len(traj["time"])
    print(f"Executing {duration_s:.1f} s trajectory ({n_points} setpoints)...")

    start_time = time.time()
    for i in range(n_points):
        x = float(traj["pos_ned"][i, 0])
        y = float(traj["pos_ned"][i, 1])
        z = float(traj["pos_ned"][i, 2])
        yaw_deg = float(np.rad2deg(traj["yaw_rad"][i]))

        await drone.offboard.set_position_ned(PositionNedYaw(x, y, z, yaw_deg))
        await asyncio.sleep(dt)

        if i % 100 == 0:
            elapsed = time.time() - start_time
            print(f"  [{elapsed:.1f}s / {duration_s:.1f}s] Pos: North={x:.1f}m, East={y:.1f}m, AGL={-z:.1f}m")

    print("Trajectory completed. Returning to launch & landing...")
    try:
        await drone.offboard.stop()
        await drone.action.land()
    except Exception:
        pass

    await asyncio.sleep(3.0)
    print("Disarming vehicle...")
    try:
        await drone.action.disarm()
    except Exception:
        pass

    await asyncio.sleep(2.0)
    print("Stopping PX4 SITL...")
    px4_proc.terminate()
    try:
        px4_proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        px4_proc.kill()

    # Find the newly created .ulg log
    after_logs = set()
    for root, _, files in os.walk(log_dir):
        for f in files:
            if f.endswith(".ulg"):
                after_logs.add(os.path.join(root, f))

    new_logs = list(after_logs - before_logs)
    if new_logs:
        new_log = sorted(new_logs, key=os.path.getmtime)[-1]
        print(f"-> Captured authentic flight log: {new_log} ({os.path.getsize(new_log)/(1024*1024):.2f} MB)")
        return new_log
    else:
        # Fallback to newest file in log_dir
        all_logs = sorted([os.path.join(r, f) for r, _, fs in os.walk(log_dir) for f in fs if f.endswith(".ulg")], key=os.path.getmtime)
        return all_logs[-1] if all_logs else ""


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Run live PX4 SITL mission flight.")
    parser.add_argument("--scenario", type=int, default=1, help="Scenario ID (1-10)")
    parser.add_argument("--duration", type=float, default=30.0, help="Duration in seconds (default: 30.0)")
    args = parser.parse_args()

    asyncio.run(run_sitl_flight(args.scenario, duration_s=args.duration))
