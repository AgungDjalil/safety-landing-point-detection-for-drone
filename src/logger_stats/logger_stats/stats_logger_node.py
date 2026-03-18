#!/usr/bin/env python3
"""
stats_logger_node.py
Subscribes to:
  - /segmentation_stats        → segmentation_stats.csv  (file terpisah)
  - /landing_circle_stats      → landing_and_center.csv  (gabung dengan center)
  - /safe_circle_center_coords → landing_and_center.csv  (gabung dengan stats)

Format /landing_circle_stats yang di-parse:
  computation_time: 12.34 ms
  valid_points: 4521
  valid_percentage: 98.12 %
  plane_size: 312
  outlier_size: 83
  unique_landing_circles: 3
  landing_circle_locations:
    [1] x=1.234000 y=0.567000 z=0.012000 (odom)
    [2] x=3.100000 y=-1.200000 z=0.015000 (odom)
    [3] x=5.678000 y=2.345000 z=0.010000 (odom)

Baris ke CSV hanya ditulis apabila ada landing circle baru yang belum
pernah tersimpan sebelumnya (dedup berdasarkan koordinat lc_N_x/y/z).
"""

import re
import csv
import math
import os
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped


# ── helpers ────────────────────────────────────────────────────────────────────

def parse_stats(data: str) -> dict:
    """Parse 'key: value unit\nkey: value unit' → dict[str, str].
    Hanya untuk field skalar (angka)."""
    result = {}
    for line in data.strip().splitlines():
        line = line.strip()
        if not line:
            continue
        m = re.match(r'^([^:]+):\s*([\d.eE+\-]+)', line)
        if m:
            result[m.group(1).strip()] = m.group(2).strip()
    return result


def parse_landing_stats(data: str) -> dict:
    """
    Parse lengkap output /landing_circle_stats.

    Mengembalikan dict dengan key:
      computation_time, valid_points, valid_percentage,
      plane_size, outlier_size,
      unique_landing_circles,
      lc_{n}_x, lc_{n}_y, lc_{n}_z, lc_{n}_frame   (n = 1, 2, 3, ...)
    """
    result = {}

    scalar_keys = {
        "computation_time",
        "valid_points",
        "valid_percentage",
        "plane_size",
        "outlier_size",
        "unique_landing_circles",
    }
    for line in data.strip().splitlines():
        line = line.strip()
        if not line:
            continue
        m = re.match(r'^([^:]+):\s*([\d.eE+\-]+)', line)
        if m:
            key = m.group(1).strip()
            if key in scalar_keys:
                result[key] = m.group(2).strip()

    # Format tiap baris: "  [N] x=1.234 y=0.567 z=0.012 (frame_id)"
    lc_pattern = re.compile(
        r'\[(\d+)\]\s+'
        r'x=([\d.eE+\-]+)\s+'
        r'y=([\d.eE+\-]+)\s+'
        r'z=([\d.eE+\-]+)\s+'
        r'\(([^)]+)\)'
    )
    for line in data.splitlines():
        m = lc_pattern.search(line)
        if m:
            n = m.group(1)
            result[f"lc_{n}_x"]     = m.group(2)
            result[f"lc_{n}_y"]     = m.group(3)
            result[f"lc_{n}_z"]     = m.group(4)
            result[f"lc_{n}_frame"] = m.group(5)

    return result


def extract_lc_coords(fields: dict) -> list:
    """
    Ekstrak semua koordinat (x, y, z) landing circle dari dict hasil parse.
    Mengembalikan list of tuple, diurutkan sesuai nomor index (1-based).
    """
    coords = []
    n = 1
    while True:
        x_str = fields.get(f"lc_{n}_x")
        y_str = fields.get(f"lc_{n}_y")
        z_str = fields.get(f"lc_{n}_z")
        if x_str is None:
            break
        try:
            coords.append((float(x_str), float(y_str), float(z_str)))
        except ValueError:
            pass
        n += 1
    return coords


def is_coord_known(coord, known, tol=1e-4):
    """
    Cek apakah koordinat sudah ada di list known.
    Toleransi default 0.1 mm untuk menangani floating-point string round-trip.
    """
    cx, cy, cz = coord
    for kx, ky, kz in known:
        dist = math.sqrt((cx - kx) ** 2 + (cy - ky) ** 2 + (cz - kz) ** 2)
        if dist < tol:
            return True
    return False


def build_lc_columns(max_circles: int) -> list:
    """Buat daftar nama kolom untuk N landing circles."""
    cols = []
    for n in range(1, max_circles + 1):
        cols += [f"lc_{n}_x", f"lc_{n}_y", f"lc_{n}_z", f"lc_{n}_frame"]
    return cols


class StatsLoggerNode(Node):
    def __init__(self):
        super().__init__("stats_logger")

        # ── parameter ─────────────────────────────────────────────────────────
        self.declare_parameter("output_dir", str(Path.home() / "ros2_logs"))
        self.declare_parameter("max_landing_circles", 10)

        output_dir   = Path(self.get_parameter("output_dir").value)
        self._max_lc = self.get_parameter("max_landing_circles").value
        output_dir.mkdir(parents=True, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        # ── file 1: segmentation_stats ────────────────────────────────────────
        seg_path = output_dir / f"segmentation_stats_{timestamp}.csv"
        self._seg_file   = open(seg_path, "w", newline="")
        self._seg_writer = csv.writer(self._seg_file)
        self._seg_header_written = False

        # ── file 2: landing_circle_stats + safe_circle_center_coords ─────────
        land_path = output_dir / f"landing_and_center_{timestamp}.csv"
        self._land_file   = open(land_path, "w", newline="")
        self._land_writer = csv.writer(self._land_file)
        self._land_header_written = False

        # Daftar koordinat (x, y, z) yang sudah pernah ditulis ke CSV.
        # Dipakai untuk dedup: koordinat yang sama tidak ditulis ulang.
        self._saved_lc_coords = []

        # buffer sinkronisasi antara dua topic
        self._latest_landing_stats = None
        self._latest_center        = None

        # ── subscriptions ─────────────────────────────────────────────────────
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
            f"  segmentation   → {seg_path}\n"
            f"  landing+center → {land_path}\n"
            f"  max_landing_circles = {self._max_lc}"
        )

    # ── callback: /segmentation_stats ────────────────────────────────────────
    def _cb_segmentation_stats(self, msg: String):
        fields = parse_stats(msg.data)
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

    # ── callback: /landing_circle_stats ──────────────────────────────────────
    def _cb_landing_stats(self, msg: String):
        fields = parse_landing_stats(msg.data)
        fields["ros_time"] = self._ros_time()
        self._latest_landing_stats = fields
        self._try_flush_landing()

    # ── callback: /safe_circle_center_coords ─────────────────────────────────
    def _cb_center_coords(self, msg: PointStamped):
        self._latest_center = {
            "center_x":        str(msg.point.x),
            "center_y":        str(msg.point.y),
            "center_z":        str(msg.point.z),
            "center_frame_id": msg.header.frame_id,
        }
        self._try_flush_landing()

    # ── flush: tulis ke CSV hanya jika ada koordinat baru ────────────────────
    def _try_flush_landing(self):
        if self._latest_landing_stats is None or self._latest_center is None:
            return

        s = self._latest_landing_stats
        c = self._latest_center

        # Ekstrak semua koordinat dari pesan yang masuk
        incoming_coords = extract_lc_coords(s)

        # Filter: hanya koordinat yang belum pernah disimpan ke CSV
        new_coords = [
            coord for coord in incoming_coords
            if not is_coord_known(coord, self._saved_lc_coords)
        ]

        if not new_coords:
            # Tidak ada koordinat baru — skip, tidak tulis ke CSV
            self.get_logger().debug(
                f"Skip: semua koordinat sudah tersimpan "
                f"(saved={len(self._saved_lc_coords)}, "
                f"incoming={len(incoming_coords)})"
            )
            self._latest_landing_stats = None
            self._latest_center        = None
            return

        # ── Ada koordinat baru → tulis satu baris ke CSV ──────────────────
        # Hanya kolom yang relevan: computation_time, valid_points, plane_size
        # (valid_percentage, outlier_size, center_x/y/z dihapus — tidak relevan)
        stats_keys  = [
            "computation_time",
            "valid_points",
            "plane_size",
        ]
        # center_frame_id tetap disimpan sebagai referensi frame saja
        center_keys = ["center_frame_id"]
        lc_keys     = build_lc_columns(self._max_lc)
        count_col   = ["count"]

        if not self._land_header_written:
            self._land_writer.writerow(
                ["ros_time"] + stats_keys + center_keys + lc_keys + count_col
            )
            self._land_header_written = True

        count_val = s.get("unique_landing_circles", "")

        row = (
            [s.get("ros_time", "")]
            + [s.get(k, "") for k in stats_keys]
            + [c.get(k, "") for k in center_keys]
            + [s.get(k, "") for k in lc_keys]
            + [count_val]
        )
        self._land_writer.writerow(row)
        self._land_file.flush()

        # Catat koordinat baru ke daftar yang sudah tersimpan
        self._saved_lc_coords.extend(new_coords)

        self.get_logger().info(
            f"CSV ditulis: {len(new_coords)} landing circle baru "
            f"(total tersimpan={len(self._saved_lc_coords)})"
        )

        # reset buffer
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