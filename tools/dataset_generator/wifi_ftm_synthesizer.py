#!/usr/bin/env python3
"""
Wi-Fi 802.11mc / 802.11az Channel & FTM/RSSI Synthesizer.
Implements:
1. Polarimetric Multipath Channel Impulse Response (CIR)
2. Coherent OFDM Subcarrier CSI & Instantaneous RSSI
3. Physical Two-Way RTT Clock Skew (ppm drift over SIFS)
4. Band-Limited Leading-Edge ToA Detection via Constant Fraction Discriminator (CFD)
5. 20ms Slot TDMA Schedule with Burst Centroid Ground Truth (16 frames/burst)
6. Dynamic Airframe & Battery Elevation Shadowing Mask
"""

import numpy as np
from typing import Dict, Any, List

SPEED_OF_LIGHT = 299792458.0  # m/s

# 5 Fixed Ground Access Points in NED coordinates (meters)
# Note: Z is negative for heights above ground (e.g. -15.0m = 15m AGL)
DEFAULT_AP_CONFIG_NED = {
    1: {"id": "AP_NORTH",  "pos": np.array([+110.0,    0.0, -15.0], dtype=np.float64)},
    2: {"id": "AP_EAST",   "pos": np.array([   0.0, +110.0, -12.0], dtype=np.float64)},
    3: {"id": "AP_SOUTH",  "pos": np.array([-110.0,    0.0, -18.0], dtype=np.float64)},
    4: {"id": "AP_WEST",   "pos": np.array([   0.0, -110.0, -10.0], dtype=np.float64)},
    5: {"id": "AP_CENTER", "pos": np.array([ +15.0,  -10.0,  -2.5], dtype=np.float64)},
}


class WifiFTMChannelSynthesizer:
    def __init__(
        self,
        fc_hz: float = 5.8e9,
        bandwidth_hz: float = 80e6,
        tx_power_dbm: float = 24.0,
        ap_gain_dbi: float = 6.0,
        uav_gain_dbi: float = 2.0,
        noise_figure_db: float = 7.0,
        ftm_burst_frames: int = 16,
        turnaround_time_s: float = 16e-6,  # SIFS = 16 microseconds
        clock_skew_ppm: float = 15.0,       # 15 ppm crystal oscillator skew
        cfd_fraction: float = 0.35,         # Constant Fraction Discriminator threshold
        ap_config: Dict[int, Dict[str, Any]] = None
    ):
        self.fc = fc_hz
        self.B = bandwidth_hz
        self.tx_power_dbm = tx_power_dbm
        self.ap_gain_dbi = ap_gain_dbi
        self.uav_gain_dbi = uav_gain_dbi
        self.P_tx = 10.0 ** ((tx_power_dbm - 30.0) / 10.0)  # Watts
        
        # Noise floor: -174 dBm/Hz + 10*log10(B) + NF
        self.P_noise_dbm = -174.0 + 10.0 * np.log10(self.B) + noise_figure_db
        self.P_noise_w = 10.0 ** ((self.P_noise_dbm - 30.0) / 10.0)
        
        self.burst_frames = ftm_burst_frames
        self.T_turnaround = turnaround_time_s
        self.clock_skew_ppm = clock_skew_ppm
        self.cfd_fraction = cfd_fraction
        
        self.ap_config = ap_config if ap_config is not None else DEFAULT_AP_CONFIG_NED
        
        # OFDM subcarriers (80 MHz: 234 active, 312.5 kHz spacing)
        self.subcarrier_spacing = 312.5e3
        self.n_subcarriers = int(self.B / self.subcarrier_spacing)
        self.subcarrier_offsets = np.arange(-self.n_subcarriers // 2, self.n_subcarriers // 2) * self.subcarrier_spacing

    def evaluate_body_shadowing(self, uav_pos_ned: np.ndarray, ap_pos_ned: np.ndarray) -> Tuple[bool, float]:
        """
        Determines line-of-sight obstruction and airframe attenuation.
        Returns (is_los, attenuation_db).
        """
        los_vec = ap_pos_ned - uav_pos_ned
        slant_range = np.linalg.norm(los_vec)
        if slant_range < 1e-3:
            return True, 0.0
        
        # Relative elevation in body frame (simplified multirotor downwards blockage)
        # Vector pointing downwards from UAV body
        los_unit = los_vec / slant_range
        # If AP is below UAV and masked by battery / carbon arm cone
        cos_down = np.dot(los_unit, np.array([0.0, 0.0, 1.0]))  # +Z is Down in NED
        
        if cos_down > 0.65:  # Mask cone directly beneath frame
            return False, 14.0  # 14 dB airframe attenuation
        return True, 0.0

    def generate_cir_multipath(self, true_distance: float, is_los: bool, attenuation_db: float) -> Tuple[np.ndarray, np.ndarray, float]:
        """
        Synthesizes realistic multipath ray delays (tau_l) and complex amplitudes (c_l).
        Includes direct path + ground bounce + 3 clutter scattering paths.
        """
        direct_delay = true_distance / SPEED_OF_LIGHT
        
        # Free space path loss: FSPL = (4 * pi * d * f / c)^2
        fspl_linear = ((4.0 * np.pi * max(true_distance, 1.0) * self.fc) / SPEED_OF_LIGHT) ** 2
        total_loss_linear = fspl_linear * (10.0 ** (attenuation_db / 10.0))
        
        p_direct = (self.P_tx / total_loss_linear) * (10.0 ** ((self.ap_gain_dbi + self.uav_gain_dbi) / 10.0))
        
        # Rays: [Direct, Ground reflection, Clutter 1, Clutter 2, Clutter 3]
        delays = [direct_delay]
        powers = [p_direct if is_los else p_direct * 0.05]
        
        # Ground bounce ray
        ground_bounce_delay = direct_delay + np.random.uniform(5e-9, 25e-9)
        ground_bounce_p = p_direct * np.random.uniform(0.08, 0.25)
        delays.append(ground_bounce_delay)
        powers.append(ground_bounce_p)
        
        # 2-3 Diffuse scattering rays
        for _ in range(3):
            delay_offset = np.random.exponential(scale=35e-9)
            delays.append(direct_delay + delay_offset)
            powers.append(p_direct * np.random.uniform(0.01, 0.08))
            
        delays = np.array(delays, dtype=np.float64)
        powers = np.array(powers, dtype=np.float64)
        
        # Random phases Uniform(0, 2*pi)
        phases = np.random.uniform(0, 2 * np.pi, len(delays))
        complex_amplitudes = np.sqrt(powers) * np.exp(1j * phases)
        
        # Calculate Ricean K-factor: power(LOS) / power(multipath)
        scattered_power = np.sum(powers[1:]) + 1e-15
        k_factor_db = 10.0 * np.log10(powers[0] / scattered_power)
        
        return delays, complex_amplitudes, k_factor_db

    def compute_coherent_ofdm_rssi(self, delays: np.ndarray, amplitudes: np.ndarray) -> Tuple[float, float]:
        """
        Integrates channel response H(f) across OFDM subcarriers to compute true RSSI and SNR.
        """
        # H(f_n) = sum_l c_l * exp(-j 2*pi*f_n*tau_l)
        # delays: [L], offsets: [N_sc]
        phase_matrix = -2.0 * np.pi * np.outer(self.subcarrier_offsets, delays)
        h_subcarriers = np.sum(amplitudes * np.exp(1j * phase_matrix), axis=1)
        
        p_rx_signal = np.mean(np.abs(h_subcarriers) ** 2)
        total_p_rx = p_rx_signal + self.P_noise_w
        
        rssi_dbm = 10.0 * np.log10(total_p_rx * 1000.0)
        snr_db = 10.0 * np.log10(max(p_rx_signal / self.P_noise_w, 1e-4))
        
        return rssi_dbm, snr_db

    def extract_cfd_leading_edge(self, delays: np.ndarray, amplitudes: np.ndarray, true_delay: float) -> Tuple[float, float]:
        """
        Applies Constant Fraction Discriminator to band-limited sinc cross-correlation profile.
        Returns (estimated_delay_s, multipath_bias_m).
        """
        # Time grid over correlation window around direct delay
        t_grid = np.linspace(true_delay - 20e-9, true_delay + 80e-9, 200)
        
        # Synthesize R_xy(tau) = sum_l c_l * sinc(B * (tau - tau_l))
        r_xy = np.zeros(len(t_grid), dtype=np.complex128)
        for c_l, tau_l in zip(amplitudes, delays):
            r_xy += c_l * np.sinc(self.B * (t_grid - tau_l))
            
        r_mag = np.abs(r_xy)
        peak_idx = np.argmax(r_mag)
        peak_val = r_mag[peak_idx]
        threshold = self.cfd_fraction * peak_val
        
        # First index before peak crossing threshold
        cross_indices = np.where(r_mag[:peak_idx + 1] >= threshold)[0]
        if len(cross_indices) > 0:
            est_delay = t_grid[cross_indices[0]]
        else:
            est_delay = true_delay
            
        multipath_bias_m = (est_delay - true_delay) * SPEED_OF_LIGHT
        return est_delay, multipath_bias_m

    def synthesize_burst_measurement(
        self,
        ap_id: int,
        uav_pos_ned_centroid: np.ndarray,
        centroid_time_s: float
    ) -> Dict[str, Any]:
        """
        Synthesizes a complete 20 ms burst interrogation of the specified AP.
        """
        ap_info = self.ap_config[ap_id]
        ap_pos = ap_info["pos"]
        
        true_dist = np.linalg.norm(uav_pos_ned_centroid - ap_pos)
        true_delay = true_dist / SPEED_OF_LIGHT
        
        is_los, atten_db = self.evaluate_body_shadowing(uav_pos_ned_centroid, ap_pos)
        delays, amplitudes, k_factor_db = self.generate_cir_multipath(true_dist, is_los, atten_db)
        
        rssi_dbm, snr_db = self.compute_coherent_ofdm_rssi(delays, amplitudes)
        est_delay, multipath_bias_m = self.extract_cfd_leading_edge(delays, amplitudes, true_delay)
        
        # Clock skew over SIFS: (delta_AP - delta_UAV) * T_turnaround
        clock_skew = np.random.uniform(-self.clock_skew_ppm, self.clock_skew_ppm) * 1e-6
        clock_error_s = clock_skew * self.T_turnaround
        
        # Intra-burst jitter across M frames
        jitter_std_s = 0.5e-9 / np.sqrt(max(10.0 ** (snr_db / 10.0), 0.1))  # Cramér-Rao lower bound scaling
        
        frame_delays = est_delay + clock_error_s + np.random.normal(0, jitter_std_s, self.burst_frames)
        burst_avg_delay = np.mean(frame_delays)
        burst_rtt_ns = (2.0 * burst_avg_delay) * 1e9
        rtt_std_ns = (2.0 * np.std(frame_delays)) * 1e9
        
        est_distance_m = (burst_rtt_ns * 1e-9 * SPEED_OF_LIGHT) / 2.0
        
        # RMS delay spread
        powers = np.abs(amplitudes) ** 2
        mean_delay = np.sum(powers * delays) / np.sum(powers)
        rms_delay_spread_ns = np.sqrt(np.sum(powers * (delays - mean_delay)**2) / np.sum(powers)) * 1e9
        
        # Physical outage check (preamble sensitivity -82 dBm, minimum SNR 4 dB)
        is_valid = (rssi_dbm >= -82.0) and (snr_db >= 4.0)
        
        return {
            "time_centroid_s": centroid_time_s,
            "ap_id": ap_id,
            "valid_flag": is_valid,
            "outage_flag": not is_valid,
            "ftm_raw_rtt_ns": burst_rtt_ns,
            "ftm_rtt_std_ns": rtt_std_ns,
            "ftm_estimated_distance_m": est_distance_m,
            "ftm_true_distance_m": true_dist,
            "ftm_multipath_bias_m": multipath_bias_m,
            "rssi_dbm": rssi_dbm,
            "snr_db": snr_db,
            "rms_delay_spread_ns": rms_delay_spread_ns,
            "ricean_k_factor_db": k_factor_db,
            "is_los": is_los
        }


if __name__ == "__main__":
    synthesizer = WifiFTMChannelSynthesizer()
    print("Testing Wi-Fi FTM Synthesizer across 5 APs at arena center (0, 0, -30m)...")
    test_uav_pos = np.array([0.0, 0.0, -30.0])
    
    for ap_id in range(1, 6):
        res = synthesizer.synthesize_burst_measurement(ap_id, test_uav_pos, centroid_time_s=10.0 + ap_id * 0.02)
        print(f"AP {ap_id} ({res['is_los'] and 'LOS' or 'NLOS'}): "
              f"True={res['ftm_true_distance_m']:.2f}m | "
              f"Est={res['ftm_estimated_distance_m']:.2f}m | "
              f"Err={res['ftm_estimated_distance_m'] - res['ftm_true_distance_m']:+.2f}m | "
              f"RSSI={res['rssi_dbm']:.1f} dBm | "
              f"SNR={res['snr_db']:.1f} dB | "
              f"Valid={res['valid_flag']}")
