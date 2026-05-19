#!/usr/bin/env python3
import argparse
import csv
import json
import math
import os
import signal
import sys
import time
from datetime import datetime

import rospy
from social_msgs.msg import SocialState


class DensityMonitor:
    def __init__(self, output_dir, topic, flush_every, summary_every_sec):
        self.output_dir = output_dir
        self.topic = topic
        self.flush_every = flush_every
        self.summary_every_sec = summary_every_sec

        self.samples = []
        self.count = 0
        self.mean = 0.0
        self.m2 = 0.0
        self.min_density = float("inf")
        self.max_density = float("-inf")
        self.last_summary_write = time.time()
        self._stop_requested = False

        os.makedirs(self.output_dir, exist_ok=True)
        self.csv_path = os.path.join(self.output_dir, "density_samples.csv")
        self.summary_path = os.path.join(self.output_dir, "density_summary.json")

        self.csv_file = open(self.csv_path, "w", newline="")
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(
            [
                "wall_time_iso",
                "ros_time_sec",
                "density",
                "people_count",
                "groups_count",
                "interactions_count",
            ]
        )

        rospy.Subscriber(self.topic, SocialState, self.callback, queue_size=50)
        rospy.loginfo("Density monitor started. topic=%s output=%s", self.topic, self.output_dir)

    def callback(self, msg):
        density = float(msg.density)
        ros_time_sec = msg.header.stamp.to_sec() if msg.header.stamp else rospy.Time.now().to_sec()
        wall_time_iso = datetime.now().isoformat()
        people_count = len(msg.people.people)
        groups_count = len(msg.groups.groups)
        interactions_count = len(msg.interactions.interactions)

        self.writer.writerow(
            [
                wall_time_iso,
                "{:.6f}".format(ros_time_sec),
                "{:.6f}".format(density),
                people_count,
                groups_count,
                interactions_count,
            ]
        )

        self.samples.append(density)
        self.count += 1
        delta = density - self.mean
        self.mean += delta / self.count
        delta2 = density - self.mean
        self.m2 += delta * delta2
        self.min_density = min(self.min_density, density)
        self.max_density = max(self.max_density, density)

        if self.count % self.flush_every == 0:
            self.csv_file.flush()

        now = time.time()
        if now - self.last_summary_write >= self.summary_every_sec:
            self.write_summary()
            self.last_summary_write = now

    def _percentile(self, sorted_values, q):
        if not sorted_values:
            return 0.0
        if len(sorted_values) == 1:
            return sorted_values[0]
        idx = q * (len(sorted_values) - 1)
        lo = int(math.floor(idx))
        hi = int(math.ceil(idx))
        if lo == hi:
            return sorted_values[lo]
        frac = idx - lo
        return sorted_values[lo] * (1.0 - frac) + sorted_values[hi] * frac

    def write_summary(self):
        if self.count == 0:
            return

        values = sorted(self.samples)
        variance = self.m2 / self.count if self.count > 0 else 0.0
        stddev = math.sqrt(max(0.0, variance))

        p50 = self._percentile(values, 0.50)
        p70 = self._percentile(values, 0.70)
        p80 = self._percentile(values, 0.80)
        p90 = self._percentile(values, 0.90)
        p95 = self._percentile(values, 0.95)

        # Threshold theo thực nghiệm:
        # - on_threshold: density cao bất thường (p80)
        # - off_threshold: thấp hơn on để tạo hysteresis (p70)
        suggested_on = p80
        suggested_off = min(p70, suggested_on * 0.8)

        summary = {
            "topic": self.topic,
            "csv_path": self.csv_path,
            "generated_at_iso": datetime.now().isoformat(),
            "sample_count": self.count,
            "min": self.min_density,
            "max": self.max_density,
            "mean": self.mean,
            "stddev": stddev,
            "percentiles": {
                "p50": p50,
                "p70": p70,
                "p80": p80,
                "p90": p90,
                "p95": p95,
            },
            "suggested_thresholds": {
                "density_on_threshold": suggested_on,
                "density_off_threshold": suggested_off,
            },
        }

        with open(self.summary_path, "w") as f:
            json.dump(summary, f, indent=2)

    def close(self):
        self.write_summary()
        self.csv_file.flush()
        self.csv_file.close()
        rospy.loginfo("Density monitor stopped. Summary: %s", self.summary_path)


def parse_args():
    parser = argparse.ArgumentParser(description="Runtime density logger for social_state.")
    parser.add_argument("--topic", default="/social_state", help="SocialState topic")
    parser.add_argument(
        "--output-dir",
        default="",
        help="Output directory for CSV and summary JSON",
    )
    parser.add_argument("--flush-every", type=int, default=50, help="Flush CSV every N samples")
    parser.add_argument(
        "--summary-every-sec",
        type=float,
        default=5.0,
        help="Rewrite summary JSON every N seconds",
    )
    return parser.parse_args(rospy.myargv(argv=sys.argv)[1:])


def main():
    rospy.init_node("density_monitor", anonymous=True)
    args = parse_args()
    if args.output_dir:
        output_dir = args.output_dir
    else:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_dir = os.path.join(os.getcwd(), "runtime_logs", "density_monitor", stamp)

    monitor = DensityMonitor(
        output_dir=output_dir,
        topic=args.topic,
        flush_every=max(1, args.flush_every),
        summary_every_sec=max(1.0, args.summary_every_sec),
    )

    def handle_signal(_sig, _frame):
        monitor._stop_requested = True
        rospy.signal_shutdown("signal")

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    rate = rospy.Rate(10)
    try:
        while not rospy.is_shutdown() and not monitor._stop_requested:
            rate.sleep()
    finally:
        monitor.close()


if __name__ == "__main__":
    main()
