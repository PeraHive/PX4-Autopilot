#!/usr/bin/env python3
"""
Full Autonomous Batch Pipeline: Real PX4 SITL Flight + Native quadriga-lib Synthesizer.
Executes all 10 scenarios through the native PX4 autopilot binary, logs actual .ulg
telemetry, computes IEEE 802.11mc/az Wi-Fi FTM RTT and coherent RSSI via quadriga-lib,
and saves publication-grade HDF5 datasets.
"""

import os
import sys
import time
import asyncio

script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)

from px4_sitl_flight_runner import run_sitl_flight
from ulog_quadriga_harmonizer import process_ulog_to_dataset
from trajectory_generator import get_scenario_metadata


def run_full_pipeline(
    scenarios=None,
    flight_duration_s: float = 900.0,
    output_dir: str = "datasets"
):
    if scenarios is None:
        scenarios = list(range(1, 11))

    os.makedirs(output_dir, exist_ok=True)
    metadata_all = get_scenario_metadata()

    print("=" * 75)
    print("AUTONOMOUS PX4 SITL (SIH 250 Hz) + NATIVE quadriga-lib PIPELINE")
    print(f"Target duration: {flight_duration_s:.1f} s ({flight_duration_s / 60.0:.1f} min) per flight")
    print(f"Total Scenarios: {len(scenarios)}")
    print(f"Output directory: {output_dir}")
    print("=" * 75)

    pipeline_start = time.time()
    generated_files = []

    for sid in scenarios:
        meta = metadata_all[sid]
        print(f"\n>>> Starting Pipeline for Scenario {sid:02d}: {meta['name']}")
        print(f"    Dynamics: {meta['description']}")

        t0 = time.time()
        # 1. Fly mission in native PX4 SITL and capture .ulg log
        ulg_path = asyncio.run(run_sitl_flight(sid, duration_s=flight_duration_s))
        if not ulg_path or not os.path.exists(ulg_path):
            print(f"ERROR: Failed to capture .ulg file for Scenario {sid}!")
            continue

        # 2. Process ULog and synthesize RF channel via native quadriga-lib
        out_hdf5 = os.path.join(output_dir, f"dataset_flight_scenario_{sid:02d}.h5")
        process_ulog_to_dataset(
            ulog_filepath=ulg_path,
            scenario_id=sid,
            output_hdf5_filepath=out_hdf5,
            target_hz=200.0
        )

        elapsed = time.time() - t0
        generated_files.append((sid, out_hdf5, elapsed))
        print(f"=== Scenario {sid} Finished in {elapsed:.1f}s ===")

    print("\n" + "=" * 75)
    print("ALL FLIGHT SCENARIOS COMPLETED & DATASETS SAVED")
    print(f"Total Wall Time: {(time.time() - pipeline_start):.1f} s")
    print("-" * 75)
    for sid, h5_file, el in generated_files:
        size_mb = os.path.getsize(h5_file) / (1024 * 1024)
        print(f"  - Scenario {sid:02d}: {h5_file} ({size_mb:.2f} MB, {el:.1f}s)")
    print("=" * 75)


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Full Autonomous PX4 SITL + quadriga-lib Batch Pipeline.")
    parser.add_argument("--scenario", type=int, default=None, help="Scenario ID (1-10) or omit for all")
    parser.add_argument("--duration", type=float, default=900.0, help="Flight duration in seconds (default: 900.0)")
    parser.add_argument("--outdir", type=str, default="datasets", help="Output directory")
    args = parser.parse_args()

    scenarios = [args.scenario] if args.scenario is not None else list(range(1, 11))
    run_full_pipeline(scenarios=scenarios, flight_duration_s=args.duration, output_dir=args.outdir)
