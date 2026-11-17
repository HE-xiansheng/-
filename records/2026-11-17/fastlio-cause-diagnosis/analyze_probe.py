#!/usr/bin/env python3
"""Summarize timestamp diagnostics without treating censored spans as all work."""
import argparse
import json
import re
from collections import Counter
from pathlib import Path


def distribution(values):
    values = sorted(values)
    if not values:
        return None
    return {
        name: values[round(q * (len(values) - 1))]
        for name, q in [("min", 0), ("median", .5), ("p95", .95), ("max", 1)]
    }


def analyze(path):
    text = path.read_text()
    rows, invalid = [], []
    for line in text.splitlines():
        if line.startswith("DIAG {"):
            try:
                rows.append(json.loads(line[5:]))
            except json.JSONDecodeError:
                invalid.append(line)
    start = min((r["wall"] for r in rows), default=0)
    imu = [r for r in rows if r["kind"] == "imu" and r["wall"] >= start + 3]
    mapping = [r for r in rows if r["kind"] == "mapping" and r["wall"] >= start + 3]
    spans = [r for r in rows if r["kind"] == "span"]
    windows = {}
    for label, low, high in [("before_pause", 3, 9), ("after_recovery", 16, 1000)]:
        samples = [r for r in imu if low <= r["wall"] - start < high]
        windows[label] = {
            "imu_log_count": len(samples),
            "imu_hz_from_counter": ((samples[-1]["count"] - samples[0]["count"]) /
                                    (samples[-1]["wall"] - samples[0]["wall"])) if len(samples) > 1 else None,
            "raw_age_s": distribution([r["wall"] - r["raw"] for r in samples]),
            "publish_to_rmw_receive_s": distribution([
                r["received"] - r["published"] for r in samples if r.get("received", 0) > 0]),
            "rmw_receive_to_callback_s": distribution([
                r["wall"] - r["received"] for r in samples if r.get("received", 0) > 0]),
        }
    return {
        "source": path.name,
        "rows": dict(Counter(r["kind"] for r in rows)),
        "malformed_lines": invalid,
        "detected_offsets_s": [float(v) for v in re.findall(
            r"Self sync IMU and LiDAR, time diff is ([-\d.]+)", text)],
        "imu_hz_from_counter": ((imu[-1]["count"] - imu[0]["count"]) /
                                (imu[-1]["wall"] - imu[0]["wall"])) if len(imu) > 1 else None,
        "imu_raw_age_s": distribution([r["wall"] - r["raw"] for r in imu]),
        "imu_prepublish_age_s": distribution([r["published"] - r["raw"] for r in imu]),
        "imu_publish_to_callback_s": distribution([r["wall"] - r["published"] for r in imu]),
        "imu_publish_to_rmw_receive_s": distribution([
            r["received"] - r["published"] for r in imu if r.get("received", 0) > 0]),
        "imu_rmw_receive_to_callback_s": distribution([
            r["wall"] - r["received"] for r in imu if r.get("received", 0) > 0]),
        "windows_relative_to_first_diag_wall": windows,
        "compute_s": distribution([r["compute_s"] for r in mapping]),
        "mapping_source_age_s": distribution([r["wall"] - r["source_end"] for r in mapping]),
        "slow_callback_spans": {
            name: {"count": len(values), "sum_duration_s": sum(values),
                   "duration_s": distribution(values)}
            for name in sorted({r["callback"] for r in spans})
            for values in [[r["duration"] for r in spans if r["callback"] == name]]
        },
        "span_censoring": "Only callback durations >0.025s are logged; not a distribution of all callbacks.",
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("logs", type=Path, nargs="+")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.write_text(json.dumps([analyze(p) for p in args.logs], indent=2) + "\n")
