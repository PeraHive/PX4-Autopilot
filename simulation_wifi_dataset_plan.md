# Architectural and Scientific Plan: High-Rate UAV Simulation & Wi-Fi FTM/RSSI Channel Emulation Dataset (v2.0 - Peer-Reviewed & Corrected)

## 1. Executive Summary & Objective

This document defines the peer-reviewed, mathematically consistent, and publication-grade architectural plan to generate a benchmark-quality dataset for UAV localization, sensor fusion, and attitude estimation research.

### Core Upgrades & Resolved Audit Verdicts:
1. **Scaled Arena & Physically Feasible Link Budget:** Reduced AP perimeter radius to $\pm 100\text{--}150\text{ m}$ (arena $200\text{--}300\text{ m}$) with outdoor AP EIRP set to $+28\text{ to }+30\text{ dBm}$. Received SNR is kept in realistic operational regimes ($10\text{--}35\text{ dB}$), and an explicit physical preamble detection / outage model is introduced.
2. **Unified Frame Transformations:** Strict enforcement of **North-East-Down (NED)** for local Cartesian positions and **Front-Right-Down (FRD)** for body rates/accelerations. Explicit polarimetric transformation matrix $\mathbf{R}_{\text{ENU}\leftarrow\text{NED}}$ maps poses into QuaDRiGa's native East-North-Up (ENU) frame with sign inversion prevented.
3. **Coherent Polarimetric OFDM CIR & RSSI:** Replaces heuristic decoupling with discrete-time Fourier integration of QuaDRiGa's polarimetric Jones matrices $\mathbf{M}_{i,l}$ across active OFDM subcarriers. RSSI strictly matches instantaneous multipath constructive/destructive fading.
4. **Physical FTM Clock Physics & Leading-Edge Detection:** Two-way RTT cancels absolute clock offsets; bounded residual drift is modeled from crystal oscillator ppm skew over SIFS turnaround ($T_{\text{turnaround}} = 16\,\mu\text{s}$). Band-limited cross-correlation profiles $R_{xy}(\tau)$ are processed using a physical Constant Fraction Discriminator (CFD) leading-edge search.
5. **Intra-Burst Motion & Burst Centroid Alignment:** Accounts for intra-burst UAV displacement during the $20\text{ ms}$ window ($M=16$ frames per burst) by evaluating ground-truth distances at the burst centroid timestamp $\bar{t}_k = t_k - 10\text{ ms}$, eliminating velocity-induced systematic lag.
6. **Airframe Obstruction Modeling:** Incorporates full-wave 3D embedded antenna radiation patterns (`.qdant`) or calibrated body-frame elevation masking $L_{\text{body}}(\theta_{\text{body}}, \phi_{\text{body}})$ for drone frame and battery blockage.
7. **Accurate PX4 SITL Parameters:** Replaces non-existent parameters with verified PX4 settings (`SDLOG_PROFILE = 31` or custom `logger_topics.txt`, `SDLOG_MODE = 0`, `SIM_BAT_DRAIN = 0` to disable battery starvation, and logging direct simulator topics `vehicle_local_position_groundtruth` and `vehicle_attitude_groundtruth`).
8. **11-DOF Feedforward Trajectory Control:** MAVSDK setpoints supply position, velocity, acceleration $[\ddot{x}, \ddot{y}, \ddot{z}]$, and yaw-rate $\dot{\psi}$ at $50\text{ Hz}$ to eliminate controller tracking lag.
9. **Band-Limited IMU Pre-Integration & $SO(3)$ SLERP:** Quaternions are interpolated strictly on $SO(3)$ using SLERP. High-rate $1000\text{ Hz}$ IMU signals are integrated over $5\text{ ms}$ bins to produce coning/sculling-compensated strapdown increments ($\Delta\mathbf{v}_k, \Delta\boldsymbol{\theta}_k$) without vibration aliasing.
10. **Publication-Grade Schema:** Complete metadata, lever-arm extrinsics, IMU bias random walks, Allan variance parameters, and channel quality metrics ($K$-factor, RMS delay spread).

---

## 2. System Architecture

```mermaid
graph TD
    subgraph Physics & Autopilot Layer (Tier A)
        GZ[Gazebo Sim gz_x500] <-->|Lockstep / Clock Sync| PX4[PX4 SITL Autopilot]
        PX4 -->|High-rate Sensors & State| ULOG[PX4 SDLog .ulg file]
        GZ -->|vehicle_local_position_groundtruth & vehicle_attitude_groundtruth| PX4
        MAVSDK[MAVSDK-Python 11-DOF Stream @ 50 Hz] -->|Pos, Vel, Accel, Yaw-Rate| PX4
    end

    subgraph RF Emulation Layer: QuaDRiGa & Wi-Fi FTM (Tier B)
        ULOG -->|Extract 6-DOF Ground Truth Pose & Rates| SYNC_TIME[Burst Sub-sampler & Centroid Evaluator]
        AP_CONFIG[5 Ground AP Anchors in NED] --> SYNC_TIME
        SYNC_TIME -->|NED to ENU Transform R_ENU_NED| QUADRIGA[quadriga-lib / Octave Engine]
        ANT_PATTERN[3D Airframe Polarimetric Pattern .qdant] --> QUADRIGA
        QUADRIGA -->|Polarimetric CIR Jones Matrices & Rays| FTM_ENGINE[Coherent OFDM & CFD FTM Synthesizer]
        FTM_ENGINE -->|Coherent RSSI, CFD Leading-Edge RTT, Delay Spread, Outages| RF_LOG[wifi_ftm_burst_stream.parquet]
    end

    subgraph Harmonization & Colab Pipeline
        ULOG --> HARMONIZER[Master Clock Aligner & SLERP Interpolator]
        RF_LOG --> HARMONIZER
        HARMONIZER --> HDF5[Unified Publication-Grade HDF5 Dataset]
        HDF5 --> COLAB[Google Colab Trajectory & EKF2 Benchmark Notebook]
    end
```

---

## 3. Rigorous RF Propagation & IEEE 802.11mc/az Mathematical Modeling

### 3.1 Scaled Arena Geometry & Link Budget Feasibility
To ensure received signals stay above the IEEE 802.11 preamble detection sensitivity ($\approx -82\text{ dBm}$ for $20\text{ MHz}$ preambles, $-76\text{ dBm}$ for $80\text{ MHz}$ LTFs), the operational flight arena is bounded to $200\text{ m} \times 200\text{ m}$ (altitudes up to $80\text{ m}$ AGL), keeping maximum slant ranges below $250\text{ m}$.

* **RF Carrier:** $f_c = 5.8\text{ GHz}$ (UNII-3 / ISM band).
* **Channel Bandwidths:** $B = 80\text{ MHz}$ (802.11mc) and $B = 160\text{ MHz}$ (802.11az).
* **Terrestrial AP Transmit Power:** $P_{\text{tx}} = 24\text{ dBm}$ ($250\text{ mW}$) with directional/collinear mast antenna gain $G_{\text{tx}} = 6\text{ dBi}$ ($\text{EIRP} = 30\text{ dBm}$).
* **UAV Transceiver:** $P_{\text{tx,uav}} = 18\text{ dBm}$, $G_{\text{rx,uav}} = 2\text{ dBi}$ dipole.

**Thermal Noise Floor ($P_{\text{noise}}$) with Receiver Noise Figure $\text{NF} = 7\text{ dB}$:**
$$P_{\text{noise}}(B) = -174\text{ dBm/Hz} + 10\log_{10}(B) + \text{NF} = \begin{cases} -88.0\text{ dBm} & (B = 80\text{ MHz}) \\ -85.0\text{ dBm} & (B = 160\text{ MHz}) \end{cases}$$

**Physical Packet Outage Model:**
For each burst interrogation, received power is tested against detection thresholds:
$$\text{valid\_flag} = \begin{cases} \text{True} & \text{if } \text{SNR}_{\text{preamble}} \ge 4.0\text{ dB} \text{ and } P_{\text{rx}} \ge -82.0\text{ dBm} \\ \text{False (Outage)} & \text{otherwise} \end{cases}$$

### 3.2 Polarimetric Spatio-Temporal CIR & Coherent OFDM RSSI
QuaDRiGa tracks $L$ multipath rays between AP $i$ and the UAV. For each path $l$, the complex polarimetric channel transfer coefficient is:
$$\tilde{c}_{i,l}(t_k) = \mathbf{F}_{\text{rx}}^T\!\left(\theta_{i,l}^{\text{AoA}}, \phi_{i,l}^{\text{AoA}}, \mathbf{q}_{\text{EB}}(t_k)\right) \begin{bmatrix} M_{i,l}^{\theta\theta} & M_{i,l}^{\theta\phi} \\ M_{i,l}^{\phi\theta} & M_{i,l}^{\phi\phi} \end{bmatrix} \mathbf{F}_{\text{tx}}\!\left(\theta_{i,l}^{\text{AoD}}, \phi_{i,l}^{\text{AoD}}\right) \sqrt{P_{\text{path}, i, l}(t_k)} \, e^{-j 2\pi f_c \tau_{i,l}(t_k)}$$

Where:
* $\mathbf{M}_{i,l} \in \mathbb{C}^{2\times 2}$ is the polarimetric scattering Jones matrix from QuaDRiGa.
* $\mathbf{F}_{\text{rx}} \in \mathbb{C}^{2\times 1}$ is the UAV 3D polarimetric antenna field pattern rotated into the propagation frame by the body orientation quaternion $\mathbf{q}_{\text{EB}}(t_k)$, incorporating cross-polarization tilt loss during bank angles.
* Airframe blockage: If the line-of-sight vector passes through the drone battery or carbon arm solid angle cone ($\theta_{\text{body}} \in [110^\circ, 160^\circ]$), an extra attenuation factor $L_{\text{body}} \approx 12\text{--}18\text{ dB}$ is applied to the direct path ($l=1$).

**Coherent OFDM Subcarrier CSI & Instantaneous RSSI:**
Evaluating across $N_{\text{sc}}$ active OFDM subcarriers spaced by $\Delta f = 312.5\text{ kHz}$ ($N_{\text{sc}} = 234$ for $80\text{ MHz}$; $468$ for $160\text{ MHz}$):
$$H_i(f_n, t_k) = \sum_{l=1}^{L} \tilde{c}_{i,l}(t_k) \, e^{-j 2\pi f_n \tau_{i,l}(t_k)}$$
$$\text{RSSI}_i(t_k) = 10 \log_{10} \left( \frac{P_{\text{tx}}}{N_{\text{sc}}} \sum_{n=1}^{N_{\text{sc}}} \left\vert{} H_i(f_n, t_k) \right\vert{}^2 + P_{\text{noise}}(B) \right) \quad [\text{dBm}]$$

### 3.3 Two-Way FTM Clock Drift & Constant Fraction Discriminator (CFD) RTT
1. **Clock Drift Cancellation:**
In IEEE 802.11mc/az double-sided two-way ranging with timestamps $(t_1, t_2, t_3, t_4)$:
$$\text{RTT}_i = (t_4 - t_1) - (t_3 - t_2) = (\tau_{\text{DL}} + \tau_{\text{UL}}) + (\delta_{\text{AP}} - \delta_{\text{UAV}}) T_{\text{turnaround}} + \eta_{\text{quant}} + \eta_{\text{jitter}}$$
Because $(t_4 - t_1)$ and $(t_3 - t_2)$ are evaluated on local clocks, the absolute time offset cancels. The residual clock bias is governed strictly by the oscillator frequency skew $\Delta\delta \sim \mathcal{U}(-20, +20)\text{ ppm}$ over SIFS turnaround $T_{\text{turnaround}} = 16\,\mu\text{s}$:
$$b_{\text{clock}} = \frac{c}{2} (\delta_{\text{AP}} - \delta_{\text{UAV}}) T_{\text{turnaround}} \le 0.048\text{ m} \quad (\approx 4.8\text{ cm})$$

2. **Band-Limited Leading-Edge ToA Extraction:**
The baseband cross-correlation profile $R_{xy}(\tau)$ of the sounding preamble is synthesized from QuaDRiGa's rays:
$$R_{xy}(\tau) = \sum_{l=1}^{L} \tilde{c}_{i,l}(t_k) \, \text{sinc}\!\Big(B\big(\tau - \tau_{i,l}(t_k)\big)\Big) + w(\tau), \quad w(\tau) \sim \mathcal{CN}(0, \sigma_w^2)$$
The estimated arrival time $\hat{\tau}_{\text{arrival}}$ is determined by a Constant Fraction Discriminator (CFD) with leading-edge threshold $\gamma_{\text{th}} = 0.35$:
$$\hat{\tau}_{i,\text{arrival}} = \min \left\{ \tau \in [\tau_{\text{peak}} - W, \tau_{\text{peak}}] \;\middle\vert{}\; |R_{xy}(\tau)| \ge \gamma_{\text{th}} \max_{\tau'} |R_{xy}(\tau')| \right\}$$
This naturally models realistic positive multipath range biases (when overlapping reflections distort the main lobe) without ad-hoc empirical offsets.

3. **Intra-Burst Motion Mitigation:**
Each $20\text{ ms}$ burst executes $M=16$ frame exchanges ($t_{k,m} \in [t_k - 20\text{ ms}, t_k]$). The reported FTM range is the burst sample mean $\bar{d}_{\text{FTM}, i}$. The ground-truth reference distance is evaluated at the **burst centroid timestamp**:
$$\bar{t}_k = t_k - 10\text{ ms}, \qquad d_{\text{true}, i}(\bar{t}_k) = \|\mathbf{p}_{\text{uav}}^{\text{NED}}(\bar{t}_k) + \mathbf{R}_{\text{NB}}(\bar{t}_k)\,\mathbf{l}_{\text{wifi}}^{\text{body}} - \mathbf{p}_{\text{ap}, i}^{\text{NED}}\|$$
This eliminates the $15\text{ cm}$ velocity-dependent bias present when evaluating truth at the slot boundary.

---

## 4. Coordinate Frames & Terrestrial Anchor Geometry

### 4.1 Strict Frame Definitions
* **Autopilot & Dataset Master Frame:** North-East-Down (**NED**), metric units.
  * $X = \text{North}$, $Y = \text{East}$, $Z = \text{Down}$ (altitudes above ground are strictly **negative** $Z$).
* **Body Reference Frame:** Front-Right-Down (**FRD**).
* **QuaDRiGa Native Frame:** East-North-Up (**ENU**).
  $$\mathbf{p}^{\text{ENU}} = \begin{bmatrix} y_{\text{NED}} \\ x_{\text{NED}} \\ -z_{\text{NED}} \end{bmatrix}, \qquad \mathbf{R}_{\text{ENU}\leftarrow\text{NED}} = \begin{bmatrix} 0 & 1 & 0 \\ 1 & 0 & 0 \\ 0 & 0 & -1 \end{bmatrix}$$

### 4.2 Anchor Geometry in Local NED Frame ($200\text{ m} \times 200\text{ m}$ Arena)

| Anchor | ID | $X_{\text{NED}}$ (m) [North] | $Y_{\text{NED}}$ (m) [East] | $Z_{\text{NED}}$ (m) [Down] | AGL Height (m) | Structural Description |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **AP 1** | `AP_NORTH` | $+110.0$ | $0.0$ | $-15.0$ | $+15.0\text{ m}$ | North boundary elevated mast |
| **AP 2** | `AP_EAST` | $0.0$ | $+110.0$ | $-12.0$ | $+12.0\text{ m}$ | East boundary perimeter mast |
| **AP 3** | `AP_SOUTH` | $-110.0$ | $0.0$ | $-18.0$ | $+18.0\text{ m}$ | South boundary high tower |
| **AP 4** | `AP_WEST` | $0.0$ | $-110.0$ | $-10.0$ | $+10.0\text{ m}$ | West boundary mast |
| **AP 5** | `AP_CENTER`| $+15.0$ | $-10.0$ | $-2.5$ | $+2.5\text{ m}$ | Ground-level offset anchor (vertical GDOP) |

*World Origin Alignment:* In PX4 SITL, Gazebo model spawn point $(0,0,0)$ matches the local origin. During parser execution, the GNSS local origin offset $(\Delta x_0, \Delta y_0, \Delta z_0)$ between `vehicle_local_position_groundtruth` and Gazebo world frame is logged and verified to be zero.

---

## 5. MAVSDK-Python Trajectory Control & Battery Configuration

### 5.1 11-DOF Feedforward Control (Eliminating Tracking Lag)
To avoid the $0.45\text{--}0.65\text{ s}$ control lag of position-only setpoints, trajectories are defined as analytical $C^3$-continuous splines. MAVSDK transmits setpoints at $50\text{ Hz}$ containing:
$$\mathbf{S}(t) = \big[ x(t),\, y(t),\, z(t),\, \dot{x}(t),\, \dot{y}(t),\, \dot{z}(t),\, \ddot{x}(t),\, \ddot{y}(t),\, \ddot{z}(t),\, \psi(t),\, \dot{\psi}(t) \big]$$
via `offboard.set_position_velocity_acceleration_ned()`.

### 5.2 Battery Drain Configuration
To prevent SITL low-battery forced RTL/landing during 15-minute ($900\text{ s}$) runs:
* Set PX4 parameter `SIM_BAT_DRAIN = 0.0` (disables simulated battery discharge).
* Set `SIM_BAT_MIN_PCT = 100.0`.
* Set `COM_LOW_BAT_ACT = 0` (warning only, no forced landing).

---

## 6. Verified PX4 Logging & Sensor Configuration

### 6.1 PX4 Parameters (Verified in `src/modules/logger/module.yaml`)
* `SDLOG_PROFILE = 31`: Enables bitmask:
  * Bit 0 (1): Default set
  * Bit 1 (2): Estimator replay (EKF2, full rate)
  * Bit 2 (4): Thermal calibration (high-rate raw IMU and baro)
  * Bit 3 (8): System identification (high-rate actuator and IMU)
  * Bit 4 (16): High rate (RC, attitude, rates, actuators)
  *(Sum: $1 + 2 + 4 + 8 + 16 = 31$)*
* `SDLOG_MODE = 0`: Logs strictly from arming to disarm.
* High-rate sensor FIFO: Set `SDLOG_PROFILE = 831` (enabling bits 8 & 9 for raw FIFO accel/gyro if uncalibrated high-bandwidth logging is required).

### 6.2 Target Topics in `.ulg` File
1. **Sensors (Raw & Filtered):**
   * `sensor_combined` (calibrated $\ge 200\text{ Hz}$ accel & gyro, coning/sculling integrated).
   * `sensor_mag` ($50\text{--}100\text{ Hz}$).
   * `sensor_baro` ($50\text{--}100\text{ Hz}$).
   * `sensor_gps` ($10\text{ Hz}$).
2. **Estimator Output (EKF2):**
   * `vehicle_attitude` (quaternion $\mathbf{q}_{NB}$).
   * `vehicle_angular_velocity` ($\boldsymbol{\omega}_{NB}^B$).
   * `vehicle_local_position` (NED position and velocity).
   * `estimator_status` (innovations, test ratios).
3. **True Simulation Ground Truth (from Gazebo Bridge):**
   * `vehicle_local_position_groundtruth`
   * `vehicle_attitude_groundtruth`
   * `vehicle_angular_velocity_groundtruth`

---

## 7. Signal Processing & Harmonization Pipeline

### 7.1 Master Clock Alignment
* The unified master time vector $t_k \in [0, 900.0]$ is established at a **uniform $200.0\text{ Hz}$ sampling rate** ($\Delta t = 5\text{ ms}$).
* The master clock is anchored to the native microsecond timestamps of `sensor_combined`.

### 7.2 Anti-Aliased IMU Pre-Integration & SLERP Quaternions
1. **Strapdown Pre-Integration:**
   Instead of point-sampling or linearly interpolating raw IMU data, high-rate samples ($1000\text{ Hz}$) falling within each $5\text{ ms}$ bin $[t_{k-1}, t_k]$ are integrated into coning/sculling-compensated increments:
   $$\Delta\mathbf{v}_k = \int_{t_{k-1}}^{t_k} \mathbf{a}(t)\,dt, \qquad \Delta\boldsymbol{\theta}_k = \int_{t_{k-1}}^{t_k} \boldsymbol{\omega}(t)\,dt$$
2. **Attitude Quaternion SLERP:**
   Attitudes $\mathbf{q}(t)$ (both ground truth and EKF2) are interpolated on $SO(3)$ using Spherical Linear Interpolation (`scipy.spatial.transform.Slerp`) with quaternion hemisphere sign continuity maintained ($\mathbf{q}_k \cdot \mathbf{q}_{k-1} \ge 0$).

---

## 8. Complete Publication-Grade HDF5 Dataset Schema

```text
dataset_flight_scenario_XX.h5
 │
 ├── /metadata
 │     ├── scenario_id: "Scenario_01_Seed_1001"
 │     ├── coordinate_frame_global: "NED (North-East-Down, meters)"
 │     ├── coordinate_frame_body: "FRD (Front-Right-Down)"
 │     ├── duration_sec: 900.0
 │     ├── base_frequency_hz: 200.0
 │     ├── anchor_positions_ned_m: [5 x 3 float64] (Z values negative for height above ground)
 │     ├── lever_arm_imu_to_wifi_body_m: [3 float64] (x, y, z offset in FRD)
 │     ├── lever_arm_imu_to_gnss_body_m: [3 float64] (x, y, z offset in FRD)
 │     ├── imu_noise_specs: {gyro_noise_density, accel_noise_density, gyro_random_walk, accel_random_walk}
 │     └── wifi_phy_config: {carrier_freq_hz: 5.8e9, bandwidth_hz: 80e6, tx_power_dbm: 24.0, ap_antenna_gain_dbi: 6.0, burst_frames: 16}
 │
 ├── /time (Shape: [N], float64 master clock in seconds at 200 Hz, synced to sensor_combined)
 │
 ├── /sensors_high_rate
 │     ├── accel_xyz: [N x 3] (m/s^2, FRD)
 │     ├── gyro_xyz:  [N x 3] (rad/s, FRD)
 │     ├── delta_v_xyz: [N x 3] (m/s, pre-integrated velocity increment over 5 ms)
 │     ├── delta_theta_xyz: [N x 3] (rad, coning-compensated angle increment over 5 ms)
 │     ├── mag_xyz:   [N x 3] (Gauss, FRD)
 │     ├── mag_valid_flag: [N] (bool)
 │     ├── baro_alt_m: [N]    (m)
 │     ├── baro_pressure_pa: [N] (Pa)
 │     └── baro_valid_flag: [N] (bool)
 │
 ├── /sensors_gnss (Sampled at ~10 Hz, resampled with valid_flag)
 │     ├── valid_flag: [N] (bool: True when new GNSS epoch received)
 │     ├── pos_lat_lon_alt: [N x 3] (deg, deg, m)
 │     ├── pos_ned: [N x 3] (m, GNSS local position)
 │     ├── vel_ned: [N x 3] (m/s)
 │     └── hdop_vdop: [N x 2]
 │
 ├── /wifi_ranging (TDMA 20ms slot / 100ms cycle)
 │     ├── active_ap_id: [N] (int 1..5, 0 when inactive)
 │     ├── valid_flag:   [N] (bool: True at burst centroid if SNR >= 4 dB and Prx >= -82 dBm)
 │     ├── outage_flag:  [N] (bool: True if packet dropped due to low SNR / deep fade)
 │     ├── ftm_raw_rtt_ns: [N] (float64, burst-averaged two-way RTT in nanoseconds)
 │     ├── ftm_rtt_std_ns: [N] (float64, intra-burst RTT standard deviation across 16 frames)
 │     ├── ftm_estimated_distance_m: [N] (float64, c * RTT / 2)
 │     ├── ftm_true_distance_m: [N] (float64, true distance evaluated at burst centroid timestamp)
 │     ├── ftm_multipath_bias_m: [N] (float64, leading-edge CFD bias from CIR)
 │     ├── rssi_dbm: [N] (float64, coherent OFDM subcarrier-integrated RSSI)
 │     ├── snr_db:   [N] (float64, post-processing SNR)
 │     ├── rms_delay_spread_ns: [N] (float64, ground-truth CIR RMS delay spread)
 │     ├── ricean_k_factor_db:  [N] (float64, ratio of direct path power to scattered power)
 │     └── is_los: [N] (bool: True if direct path is unobstructed by airframe / obstacles)
 │
 ├── /ekf2_estimates
 │     ├── pos_ned: [N x 3] (m)
 │     ├── vel_ned: [N x 3] (m/s)
 │     ├── quat_wxyz: [N x 4] (SLERP-interpolated unit quaternion)
 │     └── euler_rpy: [N x 3] (rad)
 │
 └── /ground_truth
       ├── pos_ned: [N x 3] (m, from vehicle_local_position_groundtruth)
       ├── vel_ned: [N x 3] (m/s)
       ├── acc_ned: [N x 3] (m/s^2)
       ├── quat_wxyz: [N x 4] (SLERP-interpolated unit quaternion, NED-to-FRD)
       ├── rates_body_xyz: [N x 3] (rad/s, FRD)
       ├── imu_accel_bias_xyz: [N x 3] (m/s^2, true Gazebo IMU accelerometer bias walk)
       └── imu_gyro_bias_xyz:  [N x 3] (rad/s, true Gazebo IMU gyroscope bias walk)
```

---

## 9. Implementation Roadmap

1. **Step 1:** Create `trajectory_generator.py` supplying 11-DOF setpoints $[x, y, z, \dot{x}, \dot{y}, \dot{z}, \ddot{x}, \ddot{y}, \ddot{z}, \psi, \dot{\psi}]$ for the 10 scenarios ($200\text{ m} \times 200\text{ m}$ arena).
2. **Step 2:** Configure PX4 SITL parameters (`SIM_BAT_DRAIN=0`, `SDLOG_PROFILE=31`, `SDLOG_MODE=0`) and launch `gz_x500` headless.
3. **Step 3:** Implement `mavsdk_flight_runner.py` to stream 11-DOF setpoints at $50\text{ Hz}$ and execute automated 15-minute runs.
4. **Step 4:** Implement `quadriga_rf_synthesizer.py` using `quadriga-lib` / Octave bridge with polarimetric Jones matrices, CFD leading-edge ToA, and burst centroid timestamps.
5. **Step 5:** Build `dataset_harmonizer.py` to perform SLERP quaternion interpolation, $5\text{ ms}$ strapdown IMU pre-integration, and export HDF5 files ready for Google Colab.
