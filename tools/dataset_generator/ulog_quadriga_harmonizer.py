#!/usr/bin/env python3
"""
ULog to Publication-Grade HDF5 Harmonizer with Native quadriga-lib Synthesizer.
Extracts real 250 Hz IMU, EKF2, and ground-truth topics from PX4 .ulg file,
synthesizes 20 ms slot TDMA 802.11mc/az Wi-Fi FTM & RSSI using quadriga-lib,
and saves a synchronized HDF5 dataset.
"""

import os
import sys
import h5py
import numpy as np
import pyulog
from scipy.spatial.transform import Slerp, Rotation as R
from typing import Dict, Any

script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)

from quadriga_rf_engine import QuadrigaRFSynthesizer, DEFAULT_AP_CONFIG_NED


def extract_topic_data(ulog: pyulog.ULog, topic_name: str) -> Dict[str, np.ndarray]:
    matches = [d for d in ulog.data_list if d.name == topic_name]
    if not matches:
        return {}
    return matches[0].data


def process_ulog_to_dataset(
    ulog_filepath: str,
    scenario_id: int,
    output_hdf5_filepath: str,
    target_hz: float = 200.0
) -> str:
    print(f"\nProcessing ULog: {ulog_filepath}")
    log = pyulog.ULog(ulog_filepath)

    # 1. Extract raw sensor and ground truth data
    sensor_comb = extract_topic_data(log, "sensor_combined")
    veh_att = extract_topic_data(log, "vehicle_attitude")
    veh_lpos = extract_topic_data(log, "vehicle_local_position")
    veh_lpos_gt = extract_topic_data(log, "vehicle_local_position_groundtruth")
    veh_att_gt = extract_topic_data(log, "vehicle_attitude_groundtruth")

    # Establish master timeline anchored to sensor_combined timestamps
    t_raw_us = sensor_comb["timestamp"]
    t0_us = t_raw_us[0]
    t_end_us = t_raw_us[-1]
    duration_s = (t_end_us - t0_us) * 1e-6

    dt = 1.0 / target_hz
    master_time = np.arange(0.0, duration_s, dt)
    N = len(master_time)
    print(f"Master Time: {N} samples @ {target_hz:.1f} Hz (Duration: {duration_s:.1f} s)")

    # 2. Resample and Pre-integrate IMU sensors
    t_comb_s = (t_raw_us - t0_us) * 1e-6
    accel_raw = np.column_stack([
        sensor_comb["accelerometer_m_s2[0]"],
        sensor_comb["accelerometer_m_s2[1]"],
        sensor_comb["accelerometer_m_s2[2]"]
    ])
    gyro_raw = np.column_stack([
        sensor_comb["gyro_rad[0]"],
        sensor_comb["gyro_rad[1]"],
        sensor_comb["gyro_rad[2]"]
    ])

    accel_200 = np.zeros((N, 3))
    gyro_200 = np.zeros((N, 3))
    for i in range(3):
        accel_200[:, i] = np.interp(master_time, t_comb_s, accel_raw[:, i])
        gyro_200[:, i] = np.interp(master_time, t_comb_s, gyro_raw[:, i])

    delta_v = accel_200 * dt
    delta_theta = gyro_200 * dt

    # 3. Ground Truth: Position, Velocity, Quaternions (SLERP)
    t_lpos_gt_s = (veh_lpos_gt["timestamp"] - t0_us) * 1e-6
    pos_gt = np.zeros((N, 3))
    vel_gt = np.zeros((N, 3))
    for i, ax in enumerate(["x", "y", "z"]):
        pos_gt[:, i] = np.interp(master_time, t_lpos_gt_s, veh_lpos_gt[ax])
        vel_gt[:, i] = np.interp(master_time, t_lpos_gt_s, veh_lpos_gt[f"v{ax}"])

    acc_gt = np.gradient(vel_gt, dt, axis=0)

    # Attitude SLERP (Filter any duplicate timestamps for SciPy Slerp)
    t_att_gt_s = (veh_att_gt["timestamp"] - t0_us) * 1e-6
    q_gt_raw = np.column_stack([
        veh_att_gt["q[1]"], veh_att_gt["q[2]"], veh_att_gt["q[3]"], veh_att_gt["q[0]"]
    ])
    _, unique_gt_idx = np.unique(t_att_gt_s, return_index=True)
    t_att_gt_clean = t_att_gt_s[unique_gt_idx]
    q_gt_clean = q_gt_raw[unique_gt_idx]

    r_gt_src = R.from_quat(q_gt_clean)
    slerp_gt = Slerp(t_att_gt_clean, r_gt_src)
    valid_times_gt = np.clip(master_time, t_att_gt_clean[0], t_att_gt_clean[-1])
    r_gt_interp = slerp_gt(valid_times_gt)
    q_xyzw = r_gt_interp.as_quat()
    quat_gt_wxyz = np.column_stack([q_xyzw[:, 3], q_xyzw[:, 0], q_xyzw[:, 1], q_xyzw[:, 2]])

    # 4. EKF2 Estimates
    t_lpos_s = (veh_lpos["timestamp"] - t0_us) * 1e-6
    pos_ekf = np.zeros((N, 3))
    vel_ekf = np.zeros((N, 3))
    for i, ax in enumerate(["x", "y", "z"]):
        pos_ekf[:, i] = np.interp(master_time, t_lpos_s, veh_lpos[ax])
        vel_ekf[:, i] = np.interp(master_time, t_lpos_s, veh_lpos[f"v{ax}"])

    t_att_s = (veh_att["timestamp"] - t0_us) * 1e-6
    q_ekf_raw = np.column_stack([
        veh_att["q[1]"], veh_att["q[2]"], veh_att["q[3]"], veh_att["q[0]"]
    ])
    _, unique_ekf_idx = np.unique(t_att_s, return_index=True)
    t_att_clean = t_att_s[unique_ekf_idx]
    q_ekf_clean = q_ekf_raw[unique_ekf_idx]

    r_ekf_src = R.from_quat(q_ekf_clean)
    slerp_ekf = Slerp(t_att_clean, r_ekf_src)
    valid_times_ekf = np.clip(master_time, t_att_clean[0], t_att_clean[-1])
    r_ekf_interp = slerp_ekf(valid_times_ekf)
    q_ekf_xyzw = r_ekf_interp.as_quat()
    quat_ekf_wxyz = np.column_stack([q_ekf_xyzw[:, 3], q_ekf_xyzw[:, 0], q_ekf_xyzw[:, 1], q_ekf_xyzw[:, 2]])
    euler_ekf = r_ekf_interp.as_euler("xyz")

    # 5. Native quadriga-lib Wi-Fi FTM / RSSI Synthesizer
    print("Synthesizing 802.11mc Wi-Fi FTM & RSSI with native quadriga-lib...")
    rf_engine = QuadrigaRFSynthesizer()
    slot_samples = int(0.02 / dt)  # 20 ms slot = 4 samples @ 200 Hz

    active_ap = np.zeros(N, dtype=np.int32)
    wifi_valid = np.zeros(N, dtype=bool)
    wifi_outage = np.zeros(N, dtype=bool)
    ftm_rtt = np.zeros(N, dtype=np.float64)
    ftm_rtt_std = np.zeros(N, dtype=np.float64)
    ftm_dist_est = np.zeros(N, dtype=np.float64)
    ftm_dist_true = np.zeros(N, dtype=np.float64)
    ftm_bias = np.zeros(N, dtype=np.float64)
    rssi = np.zeros(N, dtype=np.float64)
    snr = np.zeros(N, dtype=np.float64)
    rms_delay = np.zeros(N, dtype=np.float64)
    k_factor = np.zeros(N, dtype=np.float64)
    is_los = np.zeros(N, dtype=bool)

    for k in range(0, N - slot_samples, slot_samples):
        ap_idx = ((k // slot_samples) % 5) + 1
        active_ap[k : k + slot_samples] = ap_idx

        centroid_idx = k + slot_samples // 2
        centroid_pos = pos_gt[centroid_idx]
        centroid_time = master_time[centroid_idx]

        meas = rf_engine.evaluate_burst(ap_idx, centroid_pos, centroid_time)
        wifi_valid[centroid_idx] = meas["valid_flag"]
        wifi_outage[centroid_idx] = meas["outage_flag"]
        ftm_rtt[centroid_idx] = meas["ftm_raw_rtt_ns"]
        ftm_rtt_std[centroid_idx] = meas["ftm_rtt_std_ns"]
        ftm_dist_est[centroid_idx] = meas["ftm_estimated_distance_m"]
        ftm_dist_true[centroid_idx] = meas["ftm_true_distance_m"]
        ftm_bias[centroid_idx] = meas["ftm_multipath_bias_m"]
        rssi[centroid_idx] = meas["rssi_dbm"]
        snr[centroid_idx] = meas["snr_db"]
        rms_delay[centroid_idx] = meas["rms_delay_spread_ns"]
        k_factor[centroid_idx] = meas["ricean_k_factor_db"]
        is_los[centroid_idx] = meas["is_los"]

    # 6. Save HDF5 Archive
    os.makedirs(os.path.dirname(os.path.abspath(output_hdf5_filepath)), exist_ok=True)
    with h5py.File(output_hdf5_filepath, "w") as f:
        meta = f.create_group("metadata")
        meta.attrs["scenario_id"] = f"Scenario_{scenario_id:02d}"
        meta.attrs["coordinate_frame_global"] = "NED (North-East-Down, meters)"
        meta.attrs["coordinate_frame_body"] = "FRD (Front-Right-Down)"
        meta.attrs["duration_sec"] = float(duration_s)
        meta.attrs["base_frequency_hz"] = float(target_hz)
        meta.attrs["physics_backend"] = "PX4 SITL SIH (Simulator In Hardware)"
        meta.attrs["rf_channel_model"] = "Fraunhofer HHI quadriga-lib IEEE 802.11"
        meta.create_dataset("anchor_positions_ned_m", data=np.array([DEFAULT_AP_CONFIG_NED[i]["pos"] for i in range(1, 6)]))

        f.create_dataset("time", data=master_time, compression="gzip", compression_opts=4)

        sens = f.create_group("sensors_high_rate")
        sens.create_dataset("accel_xyz", data=accel_200, compression="gzip", compression_opts=4)
        sens.create_dataset("gyro_xyz", data=gyro_200, compression="gzip", compression_opts=4)
        sens.create_dataset("delta_v_xyz", data=delta_v, compression="gzip", compression_opts=4)
        sens.create_dataset("delta_theta_xyz", data=delta_theta, compression="gzip", compression_opts=4)

        wifi = f.create_group("wifi_ranging")
        wifi.create_dataset("active_ap_id", data=active_ap, compression="gzip")
        wifi.create_dataset("valid_flag", data=wifi_valid, compression="gzip")
        wifi.create_dataset("outage_flag", data=wifi_outage, compression="gzip")
        wifi.create_dataset("ftm_raw_rtt_ns", data=ftm_rtt, compression="gzip", compression_opts=4)
        wifi.create_dataset("ftm_rtt_std_ns", data=ftm_rtt_std, compression="gzip", compression_opts=4)
        wifi.create_dataset("ftm_estimated_distance_m", data=ftm_dist_est, compression="gzip", compression_opts=4)
        wifi.create_dataset("ftm_true_distance_m", data=ftm_dist_true, compression="gzip", compression_opts=4)
        wifi.create_dataset("ftm_multipath_bias_m", data=ftm_bias, compression="gzip", compression_opts=4)
        wifi.create_dataset("rssi_dbm", data=rssi, compression="gzip", compression_opts=4)
        wifi.create_dataset("snr_db", data=snr, compression="gzip", compression_opts=4)
        wifi.create_dataset("rms_delay_spread_ns", data=rms_delay, compression="gzip", compression_opts=4)
        wifi.create_dataset("ricean_k_factor_db", data=k_factor, compression="gzip", compression_opts=4)
        wifi.create_dataset("is_los", data=is_los, compression="gzip")

        ekf = f.create_group("ekf2_estimates")
        ekf.create_dataset("pos_ned", data=pos_ekf, compression="gzip", compression_opts=4)
        ekf.create_dataset("vel_ned", data=vel_ekf, compression="gzip", compression_opts=4)
        ekf.create_dataset("quat_wxyz", data=quat_ekf_wxyz, compression="gzip", compression_opts=4)
        ekf.create_dataset("euler_rpy", data=euler_ekf, compression="gzip", compression_opts=4)

        gt = f.create_group("ground_truth")
        gt.create_dataset("pos_ned", data=pos_gt, compression="gzip", compression_opts=4)
        gt.create_dataset("vel_ned", data=vel_gt, compression="gzip", compression_opts=4)
        gt.create_dataset("acc_ned", data=acc_gt, compression="gzip", compression_opts=4)
        gt.create_dataset("quat_wxyz", data=quat_gt_wxyz, compression="gzip", compression_opts=4)

    print(f"-> Successfully saved publication dataset: {output_hdf5_filepath} ({os.path.getsize(output_hdf5_filepath)/(1024*1024):.2f} MB)")
    return output_hdf5_filepath


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Convert real PX4 ULog to HDF5 with quadriga-lib.")
    parser.add_argument("--ulog", type=str, required=True, help="Path to .ulg file")
    parser.add_argument("--scenario", type=int, default=1, help="Scenario ID (1-10)")
    parser.add_argument("--out", type=str, default="datasets/dataset_flight_scenario_01_sitl.h5", help="Output path")
    args = parser.parse_args()

    process_ulog_to_dataset(args.ulog, args.scenario, args.out)
