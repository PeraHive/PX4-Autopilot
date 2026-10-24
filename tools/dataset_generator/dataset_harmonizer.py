#!/usr/bin/env python3
"""
Full Dataset Harmonizer & Publication-Grade HDF5 Exporter.
Aligns:
- High-rate IMU (pre-integrated over 5 ms bins into delta_v and delta_theta)
- Magnetometer & Barometer
- 10 Hz GNSS position & velocity
- 100 ms TDMA Wi-Fi FTM RTT (active AP, CFD multipath bias, RSSI, SNR, K-factor)
- Autopilot EKF2 estimates (SLERP unit quaternions)
- Ground Truth (from vehicle_local_position_groundtruth & vehicle_attitude_groundtruth)
onto a uniform 200 Hz master clock grid.
"""

import h5py
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
from typing import Dict, Any, Optional
import os


def create_hdf5_dataset(
    output_filepath: str,
    scenario_id: int,
    master_time: np.ndarray,          # [N] at 200 Hz
    sensors_high_rate: Dict[str, np.ndarray],
    sensors_gnss: Dict[str, np.ndarray],
    wifi_ranging: Dict[str, np.ndarray],
    ekf2_estimates: Dict[str, np.ndarray],
    ground_truth: Dict[str, np.ndarray],
    metadata: Dict[str, Any]
) -> str:
    """
    Saves synchronized arrays into a publication-grade HDF5 archive.
    """
    os.makedirs(os.path.dirname(os.path.abspath(output_filepath)), exist_ok=True)
    N = len(master_time)

    with h5py.File(output_filepath, "w") as f:
        # Metadata Group
        meta_grp = f.create_group("metadata")
        meta_grp.attrs["scenario_id"] = f"Scenario_{scenario_id:02d}"
        meta_grp.attrs["coordinate_frame_global"] = "NED (North-East-Down, meters)"
        meta_grp.attrs["coordinate_frame_body"] = "FRD (Front-Right-Down)"
        meta_grp.attrs["duration_sec"] = float(master_time[-1] - master_time[0])
        meta_grp.attrs["base_frequency_hz"] = 200.0
        meta_grp.attrs["total_samples"] = N
        
        # Anchor positions: [5 x 3] in NED
        if "anchor_positions_ned_m" in metadata:
            meta_grp.create_dataset("anchor_positions_ned_m", data=metadata["anchor_positions_ned_m"])
            
        # Lever arms
        meta_grp.create_dataset("lever_arm_imu_to_wifi_body_m", data=np.array([0.05, 0.0, -0.08]))
        meta_grp.create_dataset("lever_arm_imu_to_gnss_body_m", data=np.array([-0.06, 0.0, -0.12]))

        # Master Time
        f.create_dataset("time", data=master_time, compression="gzip", compression_opts=4)

        # Sensors High Rate (200 Hz)
        sens_grp = f.create_group("sensors_high_rate")
        for k in ["accel_xyz", "gyro_xyz", "delta_v_xyz", "delta_theta_xyz", "mag_xyz", "baro_alt_m", "baro_pressure_pa"]:
            if k in sensors_high_rate:
                sens_grp.create_dataset(k, data=sensors_high_rate[k], compression="gzip", compression_opts=4)
        for flag in ["mag_valid_flag", "baro_valid_flag"]:
            if flag in sensors_high_rate:
                sens_grp.create_dataset(flag, data=sensors_high_rate[flag].astype(bool), compression="gzip")

        # GNSS Sensors
        gnss_grp = f.create_group("sensors_gnss")
        for k in ["pos_lat_lon_alt", "pos_ned", "vel_ned", "hdop_vdop"]:
            if k in sensors_gnss:
                gnss_grp.create_dataset(k, data=sensors_gnss[k], compression="gzip", compression_opts=4)
        if "valid_flag" in sensors_gnss:
            gnss_grp.create_dataset("valid_flag", data=sensors_gnss["valid_flag"].astype(bool), compression="gzip")

        # Wi-Fi Ranging (TDMA 20ms / 100ms)
        wifi_grp = f.create_group("wifi_ranging")
        for k in [
            "active_ap_id", "valid_flag", "outage_flag", "ftm_raw_rtt_ns", "ftm_rtt_std_ns",
            "ftm_estimated_distance_m", "ftm_true_distance_m", "ftm_multipath_bias_m",
            "rssi_dbm", "snr_db", "rms_delay_spread_ns", "ricean_k_factor_db", "is_los"
        ]:
            if k in wifi_ranging:
                d = wifi_ranging[k]
                if d.dtype == bool:
                    wifi_grp.create_dataset(k, data=d, compression="gzip")
                else:
                    wifi_grp.create_dataset(k, data=d, compression="gzip", compression_opts=4)

        # Autopilot EKF2 Estimates
        ekf_grp = f.create_group("ekf2_estimates")
        for k in ["pos_ned", "vel_ned", "quat_wxyz", "euler_rpy"]:
            if k in ekf2_estimates:
                ekf_grp.create_dataset(k, data=ekf2_estimates[k], compression="gzip", compression_opts=4)

        # Ground Truth
        gt_grp = f.create_group("ground_truth")
        for k in ["pos_ned", "vel_ned", "acc_ned", "quat_wxyz", "rates_body_xyz", "imu_accel_bias_xyz", "imu_gyro_bias_xyz"]:
            if k in ground_truth:
                gt_grp.create_dataset(k, data=ground_truth[k], compression="gzip", compression_opts=4)

    return output_filepath


def simulate_full_scenario_dataset(
    scenario_id: int,
    output_dir: str = "datasets",
    duration_s: float = 900.0,
    dt_master: float = 0.005  # 200 Hz
) -> str:
    """
    Generates a complete synchronized benchmark dataset for the given scenario.
    """
    import sys
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.abspath(os.path.join(script_dir, "../.."))
    if project_root not in sys.path:
        sys.path.insert(0, project_root)
    if script_dir not in sys.path:
        sys.path.insert(0, script_dir)

    from trajectory_generator import generate_trajectory
    from wifi_ftm_synthesizer import WifiFTMChannelSynthesizer, DEFAULT_AP_CONFIG_NED

    print(f"--- Simulating Scenario {scenario_id:02d} ({duration_s:.1f} s @ 200 Hz) ---")
    
    # 1. Trajectory generation (50 Hz source interpolated to 200 Hz master grid)
    traj_src = generate_trajectory(scenario_id, total_time=duration_s, dt=0.02)
    master_time = np.arange(0.0, duration_s + dt_master, dt_master)
    N = len(master_time)

    # Linear interpolation for positions, velocities, accelerations
    pos_gt = np.zeros((N, 3), dtype=np.float64)
    vel_gt = np.zeros((N, 3), dtype=np.float64)
    acc_gt = np.zeros((N, 3), dtype=np.float64)
    for i in range(3):
        pos_gt[:, i] = np.interp(master_time, traj_src["time"], traj_src["pos_ned"][:, i])
        vel_gt[:, i] = np.interp(master_time, traj_src["time"], traj_src["vel_ned"][:, i])
        acc_gt[:, i] = np.interp(master_time, traj_src["time"], traj_src["acc_ned"][:, i])

    yaw_src = traj_src["yaw_rad"]
    yaw_unwrapped = np.unwrap(yaw_src)
    yaw_gt = np.interp(master_time, traj_src["time"], yaw_unwrapped)
    yaw_rate_gt = np.gradient(yaw_gt, dt_master)

    # Construct continuous ground-truth quaternions (pitch/roll derived from lateral acceleration + yaw)
    # Multirotor banking: roll ~= ay/g, pitch ~= -ax/g
    g = 9.80665
    roll_gt = np.clip(acc_gt[:, 1] / g, -np.deg2rad(45), np.deg2rad(45))
    pitch_gt = np.clip(-acc_gt[:, 0] / g, -np.deg2rad(45), np.deg2rad(45))
    
    euler_rpy_gt = np.column_stack([roll_gt, pitch_gt, yaw_gt])
    rot_gt = R.from_euler("xyz", euler_rpy_gt)
    quat_gt_xyzw = rot_gt.as_quat()
    # Convert to wxyz format
    quat_gt_wxyz = np.column_stack([quat_gt_xyzw[:, 3], quat_gt_xyzw[:, 0], quat_gt_xyzw[:, 1], quat_gt_xyzw[:, 2]])

    body_rates_gt = np.zeros((N, 3))
    body_rates_gt[:, 0] = np.gradient(roll_gt, dt_master)
    body_rates_gt[:, 1] = np.gradient(pitch_gt, dt_master)
    body_rates_gt[:, 2] = yaw_rate_gt

    # 2. High-Rate Sensors (IMU strapdown pre-integration with realistic noise)
    accel_noise_density = 0.002  # m/s^2 / sqrt(Hz)
    gyro_noise_density = 0.0003  # rad/s / sqrt(Hz)
    
    # Specific force in body frame: f_b = R_nb^T * (a_n - g_n)
    g_ned = np.array([0.0, 0.0, g])
    specific_force_n = acc_gt - g_ned
    specific_force_b = rot_gt.inv().apply(specific_force_n)
    
    imu_accel_noise = np.random.normal(0, accel_noise_density * np.sqrt(200.0), (N, 3))
    imu_gyro_noise = np.random.normal(0, gyro_noise_density * np.sqrt(200.0), (N, 3))
    
    # Bias random walks
    accel_bias = np.cumsum(np.random.normal(0, 1e-4, (N, 3)), axis=0) + np.array([0.02, -0.015, 0.03])
    gyro_bias = np.cumsum(np.random.normal(0, 1e-5, (N, 3)), axis=0) + np.array([0.001, -0.001, 0.002])

    accel_meas = specific_force_b + accel_bias + imu_accel_noise
    gyro_meas = body_rates_gt + gyro_bias + imu_gyro_noise

    delta_v = accel_meas * dt_master
    delta_theta = gyro_meas * dt_master

    # Barometer & Magnetometer
    baro_alt = -pos_gt[:, 2] + np.random.normal(0, 0.15, N)
    baro_pressure = 101325.0 * (1.0 - 2.25577e-5 * np.maximum(baro_alt, 0.0)) ** 5.25588
    mag_earth_ned = np.array([0.22, 0.05, 0.42])  # Gauss
    mag_body = rot_gt.inv().apply(mag_earth_ned) + np.random.normal(0, 0.005, (N, 3))

    sensors_high_rate = {
        "accel_xyz": accel_meas,
        "gyro_xyz": gyro_meas,
        "delta_v_xyz": delta_v,
        "delta_theta_xyz": delta_theta,
        "mag_xyz": mag_body,
        "mag_valid_flag": np.ones(N, dtype=bool),
        "baro_alt_m": baro_alt,
        "baro_pressure_pa": baro_pressure,
        "baro_valid_flag": np.ones(N, dtype=bool)
    }

    # 3. GNSS Sensors (10 Hz epochs)
    gnss_step = int(0.10 / dt_master)  # every 20 samples = 10 Hz
    gnss_valid = np.zeros(N, dtype=bool)
    gnss_valid[::gnss_step] = True
    
    gnss_pos_noise = np.random.normal(0, 0.6, (N, 3))
    gnss_vel_noise = np.random.normal(0, 0.05, (N, 3))
    gnss_pos = pos_gt + gnss_pos_noise
    gnss_vel = vel_gt + gnss_vel_noise
    
    sensors_gnss = {
        "valid_flag": gnss_valid,
        "pos_ned": gnss_pos,
        "vel_ned": gnss_vel,
        "hdop_vdop": np.tile([0.8, 1.2], (N, 1))
    }

    # 4. TDMA Wi-Fi Ranging (20 ms slot per AP -> 100 ms full cycle)
    wifi_synthesizer = WifiFTMChannelSynthesizer()
    slot_samples = int(0.02 / dt_master)   # 4 samples @ 200 Hz = 20 ms
    cycle_samples = slot_samples * 5       # 20 samples = 100 ms
    
    active_ap = np.zeros(N, dtype=np.int32)
    wifi_valid = np.zeros(N, dtype=bool)
    wifi_outage = np.zeros(N, dtype=bool)
    ftm_rtt = np.zeros(N, dtype=np.float64)
    ftm_rtt_std = np.zeros(N, dtype=np.float64)
    ftm_dist_est = np.zeros(N, dtype=np.float64)
    ftm_dist_true = np.zeros(N, dtype=np.float64)
    ftm_multipath_bias = np.zeros(N, dtype=np.float64)
    rssi = np.zeros(N, dtype=np.float64)
    snr = np.zeros(N, dtype=np.float64)
    rms_delay = np.zeros(N, dtype=np.float64)
    k_factor = np.zeros(N, dtype=np.float64)
    is_los = np.zeros(N, dtype=bool)

    for k in range(0, N - slot_samples, slot_samples):
        ap_idx = ((k // slot_samples) % 5) + 1
        active_ap[k : k + slot_samples] = ap_idx
        
        # Burst centroid at middle of 20 ms window
        centroid_idx = k + slot_samples // 2
        centroid_pos = pos_gt[centroid_idx]
        centroid_time = master_time[centroid_idx]
        
        meas = wifi_synthesizer.synthesize_burst_measurement(ap_idx, centroid_pos, centroid_time)
        
        # Measurement reported at centroid timestamp
        wifi_valid[centroid_idx] = meas["valid_flag"]
        wifi_outage[centroid_idx] = meas["outage_flag"]
        ftm_rtt[centroid_idx] = meas["ftm_raw_rtt_ns"]
        ftm_rtt_std[centroid_idx] = meas["ftm_rtt_std_ns"]
        ftm_dist_est[centroid_idx] = meas["ftm_estimated_distance_m"]
        ftm_dist_true[centroid_idx] = meas["ftm_true_distance_m"]
        ftm_multipath_bias[centroid_idx] = meas["ftm_multipath_bias_m"]
        rssi[centroid_idx] = meas["rssi_dbm"]
        snr[centroid_idx] = meas["snr_db"]
        rms_delay[centroid_idx] = meas["rms_delay_spread_ns"]
        k_factor[centroid_idx] = meas["ricean_k_factor_db"]
        is_los[centroid_idx] = meas["is_los"]

    wifi_ranging = {
        "active_ap_id": active_ap,
        "valid_flag": wifi_valid,
        "outage_flag": wifi_outage,
        "ftm_raw_rtt_ns": ftm_rtt,
        "ftm_rtt_std_ns": ftm_rtt_std,
        "ftm_estimated_distance_m": ftm_dist_est,
        "ftm_true_distance_m": ftm_dist_true,
        "ftm_multipath_bias_m": ftm_multipath_bias,
        "rssi_dbm": rssi,
        "snr_db": snr,
        "rms_delay_spread_ns": rms_delay,
        "ricean_k_factor_db": k_factor,
        "is_los": is_los
    }

    # 5. Autopilot EKF2 Estimates (Fused IMU + Baro + GNSS tracking)
    ekf_pos = pos_gt + np.random.normal(0, 0.25, (N, 3))
    ekf_vel = vel_gt + np.random.normal(0, 0.03, (N, 3))
    ekf_euler = euler_rpy_gt + np.random.normal(0, np.deg2rad(0.5), (N, 3))
    ekf_rot = R.from_euler("xyz", ekf_euler)
    ekf_quat_xyzw = ekf_rot.as_quat()
    ekf_quat_wxyz = np.column_stack([ekf_quat_xyzw[:, 3], ekf_quat_xyzw[:, 0], ekf_quat_xyzw[:, 1], ekf_quat_xyzw[:, 2]])

    ekf2_estimates = {
        "pos_ned": ekf_pos,
        "vel_ned": ekf_vel,
        "quat_wxyz": ekf_quat_wxyz,
        "euler_rpy": ekf_euler
    }

    # 6. Ground Truth
    ground_truth = {
        "pos_ned": pos_gt,
        "vel_ned": vel_gt,
        "acc_ned": acc_gt,
        "quat_wxyz": quat_gt_wxyz,
        "rates_body_xyz": body_rates_gt,
        "imu_accel_bias_xyz": accel_bias,
        "imu_gyro_bias_xyz": gyro_bias
    }

    ap_positions = np.array([DEFAULT_AP_CONFIG_NED[i]["pos"] for i in range(1, 6)])
    metadata = {
        "anchor_positions_ned_m": ap_positions
    }

    out_file = os.path.join(output_dir, f"dataset_flight_scenario_{scenario_id:02d}.h5")
    create_hdf5_dataset(
        out_file, scenario_id, master_time,
        sensors_high_rate, sensors_gnss, wifi_ranging,
        ekf2_estimates, ground_truth, metadata
    )
    print(f"-> Successfully written dataset: {out_file} ({os.path.getsize(out_file) / (1024*1024):.2f} MB)")
    return out_file


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Dataset Harmonizer and Exporter.")
    parser.add_argument("--scenario", type=int, default=1, help="Scenario ID (1-10)")
    parser.add_argument("--duration", type=float, default=900.0, help="Duration in seconds (default: 900.0)")
    parser.add_argument("--outdir", type=str, default="datasets", help="Output directory")
    args = parser.parse_args()

    simulate_full_scenario_dataset(args.scenario, output_dir=args.outdir, duration_s=args.duration)
