#!/usr/bin/env python3
"""
Batch Runner for all 10 UAV Localization & Wi-Fi FTM Scenarios.
Generates complete 15-minute (900 seconds) datasets at 200 Hz.
"""

import os
import sys
import time

script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)

from dataset_harmonizer import simulate_full_scenario_dataset
from trajectory_generator import get_scenario_metadata


def run_batch_generation(output_dir: str = "datasets", duration_s: float = 900.0, scenarios=None):
    if scenarios is None:
        scenarios = list(range(1, 11))
        
    os.makedirs(output_dir, exist_ok=True)
    metadata_all = get_scenario_metadata()
    
    print("=" * 70)
    print("UAV HIGH-RATE MULTI-SENSOR + WI-FI FTM/RSSI BATCH DATASET GENERATOR")
    print(f"Target duration: {duration_s} seconds ({duration_s / 60:.1f} minutes) per flight")
    print(f"Master Sampling Rate: 200.0 Hz (5 ms intervals)")
    print(f"Output Directory: {output_dir}")
    print(f"Total Scenarios to Generate: {len(scenarios)}")
    print("=" * 70)

    start_total = time.time()
    generated_files = []

    for sid in scenarios:
        meta = metadata_all[sid]
        print(f"\n[Scenario {sid:02d}/10] {meta['name']} (Seed: {meta['seed']})")
        print(f"Dynamics: {meta['description']}")
        
        t0 = time.time()
        file_path = simulate_full_scenario_dataset(
            scenario_id=sid,
            output_dir=output_dir,
            duration_s=duration_s
        )
        elapsed = time.time() - t0
        generated_files.append((sid, file_path, elapsed))
        print(f"Completed Scenario {sid} in {elapsed:.1f}s")

    print("\n" + "=" * 70)
    print("BATCH GENERATION COMPLETED SUCCESSFULLY")
    print(f"Total Wall Time: {(time.time() - start_total):.1f} seconds")
    print("-" * 70)
    for sid, path, dur in generated_files:
        size_mb = os.path.getsize(path) / (1024 * 1024)
        print(f"  - Scenario {sid:02d}: {path} ({size_mb:.2f} MB, {dur:.1f}s)")
    print("=" * 70)


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Batch generate 10 UAV flight datasets.")
    parser.add_argument("--outdir", type=str, default="datasets", help="Output directory")
    parser.add_argument("--duration", type=float, default=900.0, help="Duration in seconds (default: 900)")
    parser.add_argument("--scenario", type=int, default=None, help="Generate single scenario (1-10) or all if omitted")
    args = parser.parse_args()

    scenarios = [args.scenario] if args.scenario is not None else list(range(1, 11))
    run_batch_generation(output_dir=args.outdir, duration_s=args.duration, scenarios=scenarios)
