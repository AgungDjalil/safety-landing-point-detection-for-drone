#!/usr/bin/env python3
"""
stats_logger_node.py
Subscribes to:
  - /segmentation_stats        → segmentation_stats.csv  (file terpisah)
  - /landing_circle_stats      → landing_and_center.csv  (gabung dengan center)
  - /safe_circle_center_coords → landing_and_center.csv  (gabung dengan stats)
"""

import re
import csv
import os
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped


# ── helpers ────────────────────────────────────────────────────────────────────

def parse_stats(data: str) -> dict:
    """Parse 'key: value unit\\nkey: value unit' → dict[str, str]."""
    result = {}
    for line in data.strip().splitlines():
        line = line.strip()
        if not line:
            continue
        # ambil key dan value pertama (abaikan satuan seperti "ms" atau "%")
        m = re.match(r'^([^:]+):\s*([\d.eE+\-]+)', line)
        if m:
            result[m.group(1).strip()] = m.group(2).strip()
    return result


class StatsLoggerNode(Node):
    def __init__(self):
        super().__init__("stats_logger")

        # ── parameter output directory ─────────────────────────────────────────
        self.declare_parameter("output_dir", str(Path.home() / "ros2_logs"))
        output_dir = Path(self.get_parameter("output_dir").value)
        output_dir.mkdir(parents=True, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        # ── file 1: segmentation_stats (RANSAC node) ───────────────────────────
        seg_path = output_dir / f"segmentation_stats_{timestamp}.csv"
        self._seg_file   = open(seg_path, "w", newline="")
        self._seg_writer = csv.writer(self._seg_file)
        self._seg_header_written = False

        # ── file 2: landing_circle_stats + safe_circle_center_coords ──────────
        land_path = output_dir / f"landing_and_center_{timestamp}.csv"
        self._land_file   = open(land_path, "w", newline="")
        self._land_writer = csv.writer(self._land_file)
        self._land_header_written = False

        # buffer: simpan stats terakhir sampai center tiba (dan sebaliknya)
        self._latest_landing_stats: dict | None = None
        self._latest_center: dict | None        = None

        # ── subscriptions ──────────────────────────────────────────────────────
        self.create_subscription(
            String, "/segmentation_stats",
            self._cb_segmentation_stats, 10)

        self.create_subscription(
            String, "/landing_circle_stats",
            self._cb_landing_stats, 10)

        self.create_subscription(
            PointStamped, "/safe_circle_center_coords",
            self._cb_center_coords, 10)

        self.get_logger().info(
            f"StatsLogger started\n"
            f"  segmentation → {seg_path}\n"
            f"  landing+center → {land_path}"
        )

    # ── callback: /segmentation_stats ─────────────────────────────────────────
    def _cb_segmentation_stats(self, msg: String):
        fields = parse_stats(msg.data)
        # kolom tetap sesuai urutan C++
        row_keys = [
            "computation_time",
            "valid_points",
            "valid_percentage",
            "plane_size",
            "outlier_size",
        ]
        if not self._seg_header_written:
            self._seg_writer.writerow(["ros_time"] + row_keys)
            self._seg_header_written = True

        row = [self._ros_time()] + [fields.get(k, "") for k in row_keys]
        self._seg_writer.writerow(row)
        self._seg_file.flush()

    # ── callback: /landing_circle_stats ───────────────────────────────────────
    def _cb_landing_stats(self, msg: String):
        fields = parse_stats(msg.data)
        fields["ros_time"] = self._ros_time()
        self._latest_landing_stats = fields
        self._try_flush_landing()

    # ── callback: /safe_circle_center_coords ──────────────────────────────────
    def _cb_center_coords(self, msg: PointStamped):
        self._latest_center = {
            "center_x":        str(msg.point.x),
            "center_y":        str(msg.point.y),
            "center_z":        str(msg.point.z),
            "center_frame_id": msg.header.frame_id,
        }
        self._try_flush_landing()

    # ── flush baris landing ketika kedua data tersedia ─────────────────────────
    def _try_flush_landing(self):
        if self._latest_landing_stats is None or self._latest_center is None:
            return  # tunggu keduanya ada

        stats_keys  = ["computation_time", "valid_points"]
        center_keys = ["center_x", "center_y", "center_z", "center_frame_id"]

        if not self._land_header_written:
            self._land_writer.writerow(
                ["ros_time"] + stats_keys + center_keys
            )
            self._land_header_written = True

        s = self._latest_landing_stats
        c = self._latest_center
        row = (
            [s.get("ros_time", "")]
            + [s.get(k, "") for k in stats_keys]
            + [c.get(k, "") for k in center_keys]
        )
        self._land_writer.writerow(row)
        self._land_file.flush()

        # reset buffer — setiap pasangan hanya ditulis sekali
        self._latest_landing_stats = None
        self._latest_center        = None

    # ── helper ────────────────────────────────────────────────────────────────
    def _ros_time(self) -> str:
        return str(self.get_clock().now().nanoseconds / 1e6)

    def destroy_node(self):
        self._seg_file.close()
        self._land_file.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = StatsLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()