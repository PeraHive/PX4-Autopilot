# PX4 SITL & QuaDRiGa Wi-Fi Ranging Dataset Generator

This toolchain generates high-fidelity, synchronized flight dynamics and 802.11mc/az Wi-Fi FTM (Fine Timing Measurement) / RSSI ranging datasets for multirotor UAVs. It combines:

1. **PX4 Autopilot SITL in SIH mode** (Simulator In Hardware / Host) running full 250 Hz rigid-body multirotor aerodynamics and lockstep physics.
2. **Fraunhofer HHI QuaDRiGa** (`quadriga-lib` C++ engine) synthesizing deterministic 3GPP/IEEE wireless channel impulse responses, multipath delay spread, Ricean fading, airframe polarimetric shadowing, and CFD leading-edge ToA estimation.

---

## 1. Quick Start: How to Run

### Prerequisites

Ensure all required Python packages are installed:

```bash
pip install "mavsdk<4" pyulog quadriga-lib h5py scipy numpy matplotlib pandas
```

Ensure PX4 SITL binary is compiled:

```bash
# From PX4-Autopilot root:
make px4_sitl_default
```

### Running a Single Flight Scenario

To run a specific scenario (e.g., Scenario 1 for 15 minutes = 900 seconds):

```bash
python3 tools/dataset_generator/run_px4_quadriga_pipeline.py --scenario 1 --duration 900.0
```

#### Arguments

* `--scenario`: Scenario ID (`1` through `10`). Each scenario features a unique trajectory profile (Lissajous, Figure-8, step responses, vertical dives, ground-hugging multipath, perimeter box, agile bank turns, dash-hover, lawnmower grid, broadband spline).
* `--duration`: Flight time in seconds (default: `900.0` for 15 minutes).
* `--hz`: Target dataset sampling rate (default: `200.0` Hz).
* `--out`: Custom output path for `.h5` file (default: `datasets/dataset_flight_scenario_<ID>.h5`).

### Batch Generation: Running All 10 Scenarios

To generate the full suite of 10 flight datasets sequentially:

```bash
for id in $(seq 1 10); do
    echo "=========================================================="
    echo "Executing Flight Scenario $id / 10 (15 Minutes Flight)..."
    echo "=========================================================="
    python3 tools/dataset_generator/run_px4_quadriga_pipeline.py --scenario $id --duration 900.0
done
```

---

## 2. Flight Scenario Catalogue (Scenarios 1 to 10)

Each scenario represents a distinct, reproducible 15-minute (900 s) flight regime designed to comprehensively span the state space of multirotor kinematics, sensor dynamics, and wireless multipath propagation conditions.

### Scenario Matrix Overview

| ID | Name | Trajectory Geometry | Altitude AGL | Dynamic Regime | RF & Localization Stress Focus |
| :---: | :--- | :--- | :---: | :--- | :--- |
| **01** | **3D Lissajous & Multi-Altitude Loops** | Quasi-periodic 3D harmonic knots | $5 - 45\text{ m}$ | Moderate continuous centripetal acceleration ($v \approx 4\text{-}7\text{ m/s}$) | **Nominal Baseline**: Uniform spatial coverage across all 5 AP coverage zones. |
| **02** | **High-Speed Figure-8s** | Lemniscate of Bernoulli | $15 - 45\text{ m}$ | Alternating roll tilt ($\pm 25^\circ$), lateral $a_y$ reversals ($v \approx 10\text{-}12\text{ m/s}$) | **Polarization Tilt Loss**: Cross-polarization discrimination (XPD) against vertical AP dipoles. |
| **03** | **Rapid Step Responses & Heading Slaloms** | High-frequency sinusoidal slalom | $10 - 30\text{ m}$ | Transient acceleration jerks, high yaw rate transitions ($\dot{\psi} > 1.2\text{ rad/s}$) | **IMU Pre-Integration & Gating**: Rapid innovation outlier rejection during kinematic jerks. |
| **04** | **Helical Ascents, Descents & Dives** | Expanding/contracting 3D helix | $10 - 70\text{ m}$ | High vertical climb/dive rates ($v_z \approx \pm 4\text{ m/s}$) | **Vertical GDOP & Elevation Pattern Nulls**: Tests vertical antenna gain taper and floor reflections. |
| **05** | **Low-Altitude Ground-Hugging** | Low-level undulating contour sweep | $2 - 8\text{ m}$ | Slow cruising close to ground plane ($v \approx 3\text{-}5\text{ m/s}$) | **Severe Multipath Stress**: Ground specular two-ray reflections, low Ricean $K$-factor, CFD ToA delay bias. |
| **06** | **Perimeter Box Patrol** | $126\text{ m} \times 126\text{ m}$ perimeter box | $20 - 40\text{ m}$ | Constant high-speed cruise ($8\text{-}10\text{ m/s}$) with 90° corner stops | **Weak-Signal & Packet Outages**: Long-range links ($\sim 180\text{ m}$), sensitivity threshold tests ($-82\text{ dBm}$). |
| **07** | **Agile High-Bank Turns** | Aggressive tight circular loops | $22 - 38\text{ m}$ | High bank angles ($>35^\circ$), high sustained lateral $g$-force ($>6\text{ m/s}^2$) | **Airframe Body Shadowing**: Carbon fiber, battery, and motors occluding direct LOS rays to opposite APs. |
| **08** | **Periodic Dash-and-Hover** | Stop-and-go radial sprint pattern | Fixed $25\text{ m}$ | Hard acceleration ($4\text{ m/s}^2$), harsh braking jerk, static hovers | **Zero-Velocity (ZUPT) & Clock Drift**: Doppler-free static anchor ranging vs. dynamic tracking. |
| **09** | **Dense 3D Lawnmower Grid Survey** | 6-lane raster survey grid | $15 - 25\text{ m}$ | Steady forward velocity with 180° hairpin turnarounds | **Radio SLAM & Spatial Fingerprinting**: Complete uniform spatial sampling of the horizontal arena plane. |
| **10** | **Broadband Pseudo-Random Spline** | Multi-frequency non-harmonic spline | $13 - 37\text{ m}$ | Non-periodic multi-frequency wideband 6-DOF excitation | **Universal Generalization**: Prevents neural networks/filters from overfitting to periodic orbits. |

---

### Detailed Scenario Descriptions

#### Scenario 01: 3D Lissajous & Multi-Altitude Loops (Nominal Baseline)
* **Mathematical Definition**: Incommensurate harmonic periods ($T_x = 90\text{ s}, T_y = 60\text{ s}, T_z = 120\text{ s}$):
  $$x(t) = 0.85 R \sin(\omega_1 t), \quad y(t) = 0.85 R \sin\left(\omega_2 t + \frac{\pi}{4}\right), \quad z_{\text{NED}}(t) = -(25.0 + 20.0 \sin(\omega_3 t))$$
* **Kinematics**: Smooth harmonic velocity ($4 - 7\text{ m/s}$), gentle attitude roll/pitch tilts ($<15^\circ$), continuous yaw tracking the velocity vector.
* **RF Propagation**: High LOS availability (>90%), moderate Ricean $K$-factor ($6 - 12\text{ dB}$), standard free-space path loss spanning $35\text{ m}$ to $185\text{ m}$.
* **Primary Evaluation Target**: Clean baseline benchmark for sensor pre-integration, nominal EKF2 convergence, and standard Wi-Fi multilateration accuracy.

#### Scenario 02: High-Speed Figure-8s (Centripetal Acceleration & Roll Tilt)
* **Mathematical Definition**: Lemniscate trajectory with period $T = 50\text{ s}$:
  $$x(t) = 0.8 R \sin(\omega t), \quad y(t) = 0.4 R \sin(2\omega t), \quad z_{\text{NED}}(t) = -(30.0 + 15.0 \cos(0.5\omega t))$$
* **Kinematics**: Periodic roll flips between $+25^\circ$ and $-25^\circ$, high sustained centripetal acceleration ($a_y$ reversals at the crossover nexus), horizontal speed peaks up to $12\text{ m/s}$.
* **RF Propagation**: Polarization misalignment between the tilted UAV dipole antenna and vertically polarized ground APs induces Cross-Polarization Discrimination (XPD) attenuation drops of $3 - 8\text{ dB}$ during banking turns.
* **Primary Evaluation Target**: Evaluating attitude-dependent antenna radiation patterns and testing estimator robustness against attitude-correlated RSSI drops.

#### Scenario 03: Rapid Step Responses & Heading Slaloms (Transient Jerks)
* **Mathematical Definition**: Harmonic slalom with high-rate direction alternations ($T = 40\text{ s}$):
  $$x(t) = 0.7 R \sin(\omega t), \quad y(t) = 0.7 R \cos(0.5\omega t), \quad z_{\text{NED}}(t) = -(20.0 + 10.0 \sin(2\omega t))$$
* **Kinematics**: Aggressive heading rate changes ($\dot{\psi} > 1.2\text{ rad/s}$), high jerk transients on lateral accelerometers, fast translational velocity transitions.
* **RF Propagation**: Rapid Doppler frequency shifts ($\pm 230\text{ Hz}$ at $5.8\text{ GHz}$ carrier), fast phase variations across OFDM subcarriers.
* **Primary Evaluation Target**: Testing Kalman filter innovation gating, high-rate IMU pre-integration accuracy ($\Delta\mathbf{v}, \Delta\boldsymbol{\theta}$), and outlier rejection during rapid maneuvers.

#### Scenario 04: Helical Ascents, Descents & Dives (High Vertical Envelope & GDOP)
* **Mathematical Definition**: Contracting/expanding cylindrical helix with deep vertical cycle ($T_{xy} = 35\text{ s}, T_z = 180\text{ s}$):
  $$x(t) = 0.65 R \cos(\omega t), \quad y(t) = 0.65 R \sin(\omega t), \quad z_{\text{NED}}(t) = -\left(10.0 + 30.0\left(1 + \sin\left(\frac{2\pi t}{180}\right)\right)\right)$$
* **Kinematics**: Broad altitude range ($10\text{ m}$ to $70\text{ m}$ AGL), sustained vertical climb/sink rates ($v_z \approx \pm 4\text{ m/s}$), continuous 360° yaw rotations.
* **RF Propagation**: Tests vertical elevation radiation patterns of ground APs (traversing elevation angles $\theta \in [-60^\circ, +60^\circ]$); evaluates Geometric Dilution of Precision (GDOP) degradation as UAV climbs far above the ground AP mast plane.
* **Primary Evaluation Target**: 3D multilateration vertical observability ($Z$-axis disambiguation) and barometer/FTM fusion under high vertical dynamics.

#### Scenario 05: Low-Altitude Ground-Hugging (Severe Multipath Stress Test)
* **Mathematical Definition**: Near-ground undulating sweep ($T = 60\text{ s}$):
  $$x(t) = 0.75 R \cos(\omega t), \quad y(t) = 0.75 R \sin(1.5\omega t), \quad z_{\text{NED}}(t) = -(5.0 + 3.0 \sin(3\omega t))$$
* **Kinematics**: Low altitude AGL ($2\text{ m}$ to $8\text{ m}$), slow forward velocity ($3 - 5\text{ m/s}$).
* **RF Propagation**: **Extreme multipath environment.** Strong specular ground reflections (two-ray ground-bounce model), short delay paths ($\Delta\tau < 15\text{ ns}$) creating destructive Rayleigh/Ricean fading, leading-edge ToA delay bias in CFD detection, and periodic NLOS dips.
* **Primary Evaluation Target**: Benchmarking multipath mitigation algorithms, ToA leading-edge estimation, and evaluating FTM ranging bias under heavy ground clutter.

#### Scenario 06: Perimeter Box Patrol (Boundary Cruising & Outage Limits)
* **Mathematical Definition**: $126\text{ m} \times 126\text{ m}$ square patrol with $120\text{ s}$ period and Gaussian-filtered transitions at the four corners:
  $$\text{Box Envelope: } x, y \in [-63\text{ m}, +63\text{ m}], \quad z_{\text{NED}}(t) = -\left(30.0 + 10.0 \sin\left(\frac{2\pi t}{200}\right)\right)$$
* **Kinematics**: Steady-state high-speed straight-line flight ($v \approx 8 - 10\text{ m/s}$) separated by sharp 90° corner maneuvers.
* **RF Propagation**: Maximum geometric separation from opposite boundary APs (distances up to $185\text{ m}$), triggering the receiver sensitivity threshold ($-82\text{ dBm}$) and causing intermittent packet outages (`outage_flag = True`).
* **Primary Evaluation Target**: Testing anchor handoff logic, localization under degraded/asymmetric AP geometry, and dead-reckoning during packet dropouts.

#### Scenario 07: Agile High-Bank Turns (Airframe Shadowing & Cross-Polarization)
* **Mathematical Definition**: Rapid, tight figure loops ($T = 30\text{ s}$):
  $$x(t) = 0.8 R \sin(\omega t), \quad y(t) = 0.5 R \sin(2\omega t), \quad z_{\text{NED}}(t) = -(30.0 + 8.0 \cos(\omega t))$$
* **Kinematics**: Maximum sustainable bank angles ($>35^\circ$), aggressive pitch transitions, high centripetal acceleration ($>6\text{ m/s}^2$).
* **RF Propagation**: Structural airframe body shadowing. When the drone banks heavily, its carbon fiber chassis, central electronics stack, and battery pack occlude the direct LOS ray to the outboard APs, inducing up to $12\text{ dB}$ of shadow attenuation and forcing the receiver to lock onto secondary NLOS reflections.
* **Primary Evaluation Target**: Evaluating Non-Line-of-Sight (NLOS) detection classifiers, shadow fading identification, and robust factor-graph optimization.

#### Scenario 08: Periodic Dash-and-Hover (Harsh Braking Jerk & ZUPT)
* **Mathematical Definition**: Stop-and-go radial sprint pattern ($45\text{ s}$ cycle):
  * **0 - 20 s (Sprint)**: Radial sprint along azimuth $\theta_k = k \cdot \frac{\pi}{3}$ with peak velocity $>12\text{ m/s}$.
  * **20 - 45 s (Stationary Hover)**: Zero velocity hold at origin ($x=0, y=0, z=-25\text{ m}$).
* **Kinematics**: Maximum longitudinal acceleration ($+4\text{ m/s}^2$) followed by hard braking jerk ($-5\text{ m/s}^2$), followed by 25 seconds of stationary hover.
* **RF Propagation**: Zero Doppler shift during hover allowing precise static crystal clock drift measurement and stationary FTM RTT variance estimation, contrasted with high Doppler spread during sprint.
* **Primary Evaluation Target**: Zero-Velocity Updates (ZUPT), online IMU accelerometer/gyro bias calibration during hover intervals, and motion-blur vs. static ranging accuracy.

#### Scenario 09: Dense 3D Lawnmower Grid Survey (Radio SLAM & Spatial Mapping)
* **Mathematical Definition**: 6-lane photogrammetry raster scan ($150\text{ s}$ period) with $180^\circ$ smoothed hairpin turns:
  $$\text{Grid Coverage: } 6 \text{ parallel survey lines across } 126\text{ m} \times 135\text{ m}, \quad z_{\text{NED}}(t) = -(20.0 + 5.0 \sin(2\pi t / 50.0))$$
* **Kinematics**: Uniform forward cruise velocity ($4\text{ m/s}$), strictly controlled lane spacing, regular periodic turnaround maneuvers.
* **RF Propagation**: Comprehensive horizontal 2D spatial sampling covering the entire arena with uniform density, creating spatial field measurements for all 5 APs.
* **Primary Evaluation Target**: Radio SLAM, 2D/3D spatial RSSI fingerprinting, Gaussian Process (GP) RF channel regression, and anchor self-calibration.

#### Scenario 10: Broadband Pseudo-Random Spline (Universal System Identification)
* **Mathematical Definition**: Superposition of non-harmonic sinusoidal frequencies with random spatial phases across all axes:
  $$x(t) = \sum_{k=1}^4 A_k \sin(2\pi f_k t + \phi_{x,k}), \quad y(t) = \sum_{k=1}^4 A_k \cos(2\pi f_k t + \phi_{y,k}), \quad z(t) = -\left(25.0 + \sum_{k=1}^4 B_k \sin(2\pi f_k t + \phi_{z,k})\right)$$
  where $f_k \in \{0.015, 0.035, 0.08, 0.14\}\text{ Hz}$.
* **Kinematics**: Non-periodic, chaotic, broadband spectral excitation across all 6 degrees of freedom simultaneously.
* **RF Propagation**: Arbitrary, uncorrelated spatial sampling preventing any phase locking with channel multipath cycles.
* **Primary Evaluation Target**: Model validation and generalizability testing. Prevents neural networks, deep Kalman filters, and state estimators from memorizing or overfitting to periodic flight patterns.

---

## 3. Pipeline Architecture

The pipeline uses a **Two-Tier Hybrid Architecture** to prevent lockstep physics starvation:

```
+-------------------------------------------------------------------------------+
|  TIER 1: Real-Time / Fast-Time PX4 SITL Flight Execution                      |
|                                                                               |
|  Trajectory Setpoints  --->  MAVSDK-Python Offboard (20 Hz)                   |
|                                    |                                          |
|                                    v                                          |
|  PX4 SITL (SIH 250 Hz) <--- Position / Velocity Lockstep Flight Controller    |
|                                    |                                          |
|                                    v                                          |
|                     Native PX4 ULog (.ulg in fs/log/)                         |
+-------------------------------------------------------------------------------+
                                     |
                                     v
+-------------------------------------------------------------------------------+
|  TIER 2: Harmonizer & QuaDRiGa RF Synthesizer (`ulog_quadriga_harmonizer.py`)  |
|                                                                               |
|  1. Parse ULog topics: `sensor_combined` (250 Hz), `vehicle_local_position`,  |
|     `vehicle_attitude`, `vehicle_local_position_groundtruth`, `vehicle_att_gt`|
|  2. Resample & Pre-integrate IMU (delta_v, delta_theta) @ 200 Hz               |
|  3. SO(3) Quaternion SLERP interpolation for attitude and position            |
|  4. 20 ms TDMA slot QuaDRiGa Channel Impulse Response (CIR) evaluation:      |
|     - AP 1 to 5 round-robin rotation (100 ms full cycle)                      |
|     - Polarimetric dipole antenna patterns with airframe shadowing            |
|     - Baseband frequency response across 234 OFDM subcarriers (80 MHz @ 5.8G) |
|     - Constant Fraction Discriminator (CFD) leading-edge ToA estimation       |
|     - SIFS clock drift with bounded crystal stability                         |
|  5. Write standardized HDF5 archive                                           |
+-------------------------------------------------------------------------------+
```

---

## 4. Ground Anchor AP Constellation

The 5 Wi-Fi Access Points are fixed in the local North-East-Down (NED) frame:

|     Anchor     | North ($X$) [m] | East ($Y$) [m] | Down ($Z$) [m] | Elevation AGL ($-Z$) [m] | Role / Placement            |
| :------------: | :---------------: | :--------------: | :--------------: | :------------------------: | :-------------------------- |
| **AP 1** |    $+110.0$    |     $0.0$     |    $-15.0$    |     $15.0\text{ m}$     | North Boundary Tower        |
| **AP 2** |      $0.0$      |    $+110.0$    |    $-12.0$    |     $12.0\text{ m}$     | East Boundary Mast          |
| **AP 3** |    $-110.0$    |     $0.0$     |    $-18.0$    |     $18.0\text{ m}$     | South High Mast             |
| **AP 4** |      $0.0$      |    $-110.0$    |    $-10.0$    |     $10.0\text{ m}$     | West Low Mast               |
| **AP 5** |     $+15.0$     |    $-10.0$    |     $-2.5$     |      $2.5\text{ m}$      | Near Ground / Near-Field AP |

---

## 5. HDF5 Dataset Specification

Every generated `.h5` file follows this clean schema:

```
dataset_flight_scenario_XX.h5
├── metadata/
│   ├── attrs: scenario_id, duration_sec, base_frequency_hz (200.0), physics_backend, rf_channel_model
│   └── anchor_positions_ned_m        [5 x 3] (Float64) Metric NED positions of AP 1-5
├── time                               [N]     (Float64) Master timestamp (seconds, 200 Hz, dt=0.005)
├── ground_truth/
│   ├── pos_ned                        [N x 3] (Float64) True UAV Position (North, East, Down in meters)
│   ├── vel_ned                        [N x 3] (Float64) True UAV Velocity (vNorth, vEast, vDown in m/s)
│   ├── acc_ned                        [N x 3] (Float64) True UAV Acceleration (m/s^2)
│   └── quat_wxyz                      [N x 4] (Float64) True Attitude Quaternion (W, X, Y, Z)
├── sensors_high_rate/
│   ├── accel_xyz                      [N x 3] (Float64) Raw IMU Accelerometer (m/s^2) in body FRD frame
│   ├── gyro_xyz                       [N x 3] (Float64) Raw IMU Gyroscope (rad/s) in body FRD frame
│   ├── delta_v_xyz                    [N x 3] (Float64) Pre-integrated Velocity Increments (m/s)
│   └── delta_theta_xyz                [N x 4] (Float64) Pre-integrated Angle Increments (rad)
├── ekf2_estimates/
│   ├── pos_ned                        [N x 3] (Float64) PX4 EKF2 Estimated Position
│   ├── vel_ned                        [N x 3] (Float64) PX4 EKF2 Estimated Velocity
│   ├── quat_wxyz                      [N x 4] (Float64) PX4 EKF2 Estimated Quaternion
│   └── euler_rpy                      [N x 3] (Float64) PX4 EKF2 Roll, Pitch, Yaw (radians)
└── wifi_ranging/
    ├── active_ap_id                   [N]     (Int32)   Active TDMA Anchor ID (1 to 5)
    ├── valid_flag                     [N]     (Bool)    True at burst centroid sample if packet decoded
    ├── outage_flag                    [N]     (Bool)    True if SNR/multipath caused burst loss
    ├── ftm_raw_rtt_ns                 [N]     (Float64) Raw Round Trip Time (nanoseconds)
    ├── ftm_rtt_std_ns                 [N]     (Float64) RTT Estimation Standard Deviation (ns)
    ├── ftm_estimated_distance_m       [N]     (Float64) Raw FTM Distance Measurement: d_est = (RTT * c) / 2
    ├── ftm_true_distance_m            [N]     (Float64) Ground Truth 3D Geometric Distance to Active AP (meters)
    ├── ftm_multipath_bias_m           [N]     (Float64) Multipath Positive Delay Bias: (d_est - d_true)
    ├── rssi_dbm                       [N]     (Float64) Coherent Received Signal Strength Indicator (dBm)
    ├── snr_db                         [N]     (Float64) Signal-to-Noise Ratio (dB)
    ├── rms_delay_spread_ns            [N]     (Float64) Channel RMS Delay Spread (ns)
    ├── ricean_k_factor_db             [N]     (Float64) Ricean K-factor (dB)
    └── is_los                         [N]     (Bool)    Line-of-Sight condition flag
```

---

## 6. Visualizing and Analyzing Data

Use the provided Jupyter notebook to visualize the 3D trajectory and Wi-Fi ranging profiles:

```bash
jupyter notebook tools/dataset_generator/flight_wifi_analysis.ipynb
```

The notebook produces:

1. **Interactive 3D Trajectory Plot**: UAV path color-coded by time/altitude with 3D ground anchor locations and elevation masts.
2. **5 AP Subplots**: For each AP, shows:
   - True 3D geometric distance over time.
   - Raw FTM ranging points ($\hat{d} = \frac{\text{RTT} \cdot c}{2}$) showing multipath bias and noise.
   - Raw RSSI ($\text{dBm}$) plotted on the secondary axis.
3. **Statistical Ranging Performance**: Error distributions, RMSE, packet reception rates, and multipath statistics.
