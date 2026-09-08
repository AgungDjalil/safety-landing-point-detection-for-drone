#!/usr/bin/env python3
"""
Rekam metrik pipeline ke CSV, satu baris per pesan.

Berlangganan:
  /segmentation_stats_cpu   (GNG)             -> <run>_gng.csv
  /segmentation_stats       (RANSAC)          -> <run>_ransac.csv
  /landing_circle_stats     (pencari titik)   -> <run>_landing.csv
                                             +  <run>_candidates.csv
  /mission_events           (waypoint_node)   -> <run>_mission.csv

Versi sebelumnya mengurai teks berformat dengan regex, dan itu gagal SENYAP.
Regex kandidatnya menuntut `(frame)` tepat sesudah `z=`, padahal formatnya
kemudian menyisipkan `hits=`, `blocked_s=`, lalu `roughness_m=` dan `score=` di
antaranya. Karena baris CSV hanya ditulis bila ada koordinat baru, dan
koordinatnya tidak pernah terparse, tidak satu baris pun pernah ditulis —
headernya pun tidak, dan tidak ada satu pun pesan galat. Berkasnya kosong,
dan tidak ada yang memberi tahu.

Karena itu tiga penerbitnya kini mengirim JSON dan berkas ini tidak lagi
mengurai apa pun sendiri: `json.loads`, lalu tulis. Menambah field di hulu
memunculkan kolom baru, bukan kegagalan senyap.

Dua aturan lain yang juga berubah, keduanya bekas penyebab berkas kosong:

  SATU BARIS PER PESAN, SELALU. Tidak ada dedup koordinat, tidak ada gerbang
  "tunggu topik lain". Analisis performa butuh deret waktu penuh; meringkas
  di sisi perekam berarti membuang data yang tidak bisa diambil ulang tanpa
  terbang lagi.

  RUN_ID WAJIB. Perbandingan GNG lawan RANSAC dilakukan bergantian, satu
  penerbangan masing-masing. Tanpa penanda penerbangan, keduanya bercampur di
  berkas yang sama dan seluruh perbandingannya sia-sia.
"""

import csv
import json
import math
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from rclpy.time import Time
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_deg_from_quaternion(q) -> float:
    """
    Arah hadap terhadap sumbu z ENU, dalam derajat.

    Ekstraksi Euler ZYX baku. Ditulis di sini alih-alih diimpor dari
    `px4_offboard_lib` karena versi di sana bekerja pada kuaternion NED dalam
    bentuk daftar [w, x, y, z], sedangkan yang masuk ke sini pesan
    `geometry_msgs/Quaternion` dalam frame ENU — memaksakan satu fungsi untuk
    keduanya justru mengundang tertukarnya konvensi frame.
    """
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return math.degrees(yaw)


class CsvSink:
    """
    Satu berkas CSV yang headernya ditentukan pesan pertama.

    Kolom diambil dari kunci baris pertama. Kunci yang muncul belakangan
    ditulis ke kolom `extra` sebagai JSON alih-alih dibuang diam-diam — sebuah
    field baru di hulu tidak boleh hilang tanpa jejak, karena itulah persis
    kegagalan yang membuat berkas ini kosong sebelumnya.
    """

    def __init__(self, path):
        """Buka `path` untuk ditulis; header menyusul saat baris pertama."""
        self.path = path
        self._f = open(path, 'w', newline='')
        self._w = csv.writer(self._f)
        self._cols = None
        self.rows = 0

    def write(self, row):
        """Tulis satu baris; kunci tak dikenal masuk kolom `extra`."""
        if self._cols is None:
            self._cols = list(row.keys())
            self._w.writerow(self._cols + ['extra'])

        known = {k: row[k] for k in self._cols if k in row}
        extra = {k: v for k, v in row.items() if k not in self._cols}
        self._w.writerow(
            [known.get(k, '') for k in self._cols]
            + [json.dumps(extra) if extra else ''])
        self._f.flush()
        self.rows += 1

    def close(self):
        """Tutup berkasnya."""
        self._f.close()


class StatsLoggerNode(Node):
    """Rekam tiap pesan metrik ke CSV, tanpa meringkas apa pun."""

    def __init__(self):
        """Buka semua sink dan berlangganan kelima topik metrik."""
        super().__init__('stats_logger')

        self.declare_parameter('output_dir', str(Path.home() / 'ros2_logs'))
        self.declare_parameter('run_id', '')

        run_id = str(self.get_parameter('run_id').value).strip()
        if not run_id:
            # Menolak jalan, bukan memakai nama default. Sebuah penerbangan
            # tidak bisa diulang dengan murah, dan data tanpa penanda
            # penerbangan tidak bisa dipisahkan lagi sesudahnya.
            raise RuntimeError(
                "Parameter 'run_id' wajib diisi -- jalankan dengan "
                "--ros-args -p run_id:=gng (atau ransac). Tanpa itu data dua "
                "penerbangan bercampur dan tidak bisa dipisahkan lagi.")

        out_dir = Path(self.get_parameter('output_dir').value)
        out_dir.mkdir(parents=True, exist_ok=True)

        # Stempel waktu ikut masuk nama berkas supaya menjalankan ulang run_id
        # yang sama tidak menimpa data penerbangan sebelumnya.
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._run_id = run_id

        self._new_sink = (
            lambda name: CsvSink(
                out_dir / ('%s_%s_%s.csv' % (run_id, stamp, name))))
        sink = self._new_sink

        self._sinks = {
            'gng':        sink('gng'),
            'ransac':     sink('ransac'),
            'landing':    sink('landing'),
            'candidates': sink('candidates'),
            'center':     sink('center'),
            'trajectory': sink('trajectory'),
        }

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=50)

        # TRANSIENT_LOCAL: waypoint_node menerbitkannya begitu, sehingga
        # perekam yang menyala terlambat tetap menerima peristiwa yang lewat.
        events_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=100)

        self.create_subscription(
            String, '/segmentation_stats_cpu',
            lambda m: self._on_json('gng', m), sensor_qos)
        self.create_subscription(
            String, '/segmentation_stats',
            lambda m: self._on_json('ransac', m), sensor_qos)
        self.create_subscription(
            String, '/landing_circle_stats', self._on_landing, sensor_qos)
        self.create_subscription(
            String, '/mission_events', self._on_mission, events_qos)
        self.create_subscription(
            PointStamped, '/safe_circle_center_coords',
            self._on_center, sensor_qos)

        # ── Lintasan drone ────────────────────────────────────────────────
        # Disampel dari TF `map -> base_link`, BUKAN dari
        # /fmu/out/vehicle_local_position. TF sudah berada dalam frame `map`
        # yang sama dengan titik pendaratan dan seluruh kandidat, jadi tidak
        # ada konversi NED->ENU yang bisa salah arah. Kecepatan bisa
        # diturunkan dengan menyelisihkan posisi saat memplot -- murah, dan
        # tidak bisa keliru frame.
        #
        # Direkam di sini, bukan di path_trail, karena data lintasan harus
        # tetap ada meski visualisasinya dimatikan: `path_trail:=false` adalah
        # opsi yang sah, dan mematikan gambar di RViz tidak boleh ikut
        # menghapus angkanya.
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('trajectory_hz', 10.0)

        self._map_frame = str(self.get_parameter('map_frame').value)
        self._base_frame = str(self.get_parameter('base_frame').value)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._last_traj_ns = None

        traj_hz = float(self.get_parameter('trajectory_hz').value)
        if traj_hz > 0.0:
            self.create_timer(1.0 / traj_hz, self._on_trajectory_tick)

        self._bad_json = 0

        self.get_logger().info(
            'StatsLogger run_id=%s -> %s\n'
            '  berlangganan: /segmentation_stats_cpu (GNG), '
            '/segmentation_stats (RANSAC), /landing_circle_stats, '
            '/mission_events, /safe_circle_center_coords\n'
            '  lintasan: TF %s -> %s pada %.1f Hz'
            % (run_id, out_dir, self._map_frame, self._base_frame, traj_hz))

    # ── helpers ──────────────────────────────────────────────────────────────

    def _base(self):
        return {'run_id': self._run_id,
                'recv_ns': self.get_clock().now().nanoseconds}

    def _decode(self, msg):
        """
        JSON -> dict, atau None bila gagal.

        Kegagalan DILAPORKAN, tidak ditelan. Berkas kosong yang tidak pernah
        mengeluh adalah alasan berkas ini ditulis ulang.
        """
        try:
            return json.loads(msg.data)
        except (ValueError, TypeError) as exc:
            self._bad_json += 1
            self.get_logger().error(
                'Muatan bukan JSON yang sah (%d kali sejauh ini): %s -- '
                'apakah node penerbitnya versi lama? Cuplikan: %.120s'
                % (self._bad_json, exc, msg.data))
            return None

    # ── callbacks ────────────────────────────────────────────────────────────

    def _on_json(self, sink_name, msg):
        payload = self._decode(msg)
        if payload is None:
            return
        row = self._base()
        row.update(self._flatten(payload))
        self._sinks[sink_name].write(row)

    def _on_mission(self, msg):
        """
        Rekam peristiwa misi, SATU BERKAS PER JENIS peristiwa.

        Transisi, abort, commit, dan ringkasan punya bentuk yang berbeda-beda,
        sedangkan CSV menuntut baris yang seragam. Menumpuknya di satu berkas
        membuat header diambil dari peristiwa pertama yang kebetulan datang,
        dan seluruh field ringkasan -- termasuk `landing_error_m`, angka utama
        penilaian misi -- terdorong ke kolom `extra` sebagai JSON. Datanya
        selamat, tapi praktis tidak bisa dibaca sebagai tabel.

        Dipisah per jenis, tiap berkas seragam dan bisa dibuka apa adanya.
        """
        payload = self._decode(msg)
        if payload is None:
            return

        kind = str(payload.get('kind', 'event'))
        name = 'mission_%s' % kind
        if name not in self._sinks:
            self._sinks[name] = self._new_sink(name)

        row = self._base()
        row.update(self._flatten(payload))
        self._sinks[name].write(row)

    def _on_landing(self, msg):
        payload = self._decode(msg)
        if payload is None:
            return

        # Kandidat dipisah ke berkasnya sendiri dalam FORMAT PANJANG: satu
        # baris per kandidat per frame. Membentangkannya menjadi kolom
        # lc_1_x, lc_2_x, ... (cara lama) membuat sebaran roughness dan skor
        # praktis tidak bisa diplot, padahal justru itu yang ingin dilihat.
        candidates = payload.pop('candidates', []) or []
        stamp_ns = payload.get('stamp_ns', '')

        row = self._base()
        row.update(self._flatten(payload))
        row['num_candidate_rows'] = len(candidates)
        self._sinks['landing'].write(row)

        for i, c in enumerate(candidates):
            crow = self._base()
            crow['stamp_ns'] = stamp_ns
            crow['candidate_index'] = i + 1
            crow.update(self._flatten(c))
            self._sinks['candidates'].write(crow)

    def _on_center(self, msg):
        row = self._base()
        row['stamp_ns'] = (int(msg.header.stamp.sec) * 1_000_000_000
                           + int(msg.header.stamp.nanosec))
        row['frame_id'] = msg.header.frame_id
        row['x'] = msg.point.x
        row['y'] = msg.point.y
        row['z'] = msg.point.z
        self._sinks['center'].write(row)

    def _on_trajectory_tick(self):
        """
        Catat satu sampel posisi drone, kalau TF-nya sudah maju.

        Stempel yang sama dilewati: lookup pada Time() selalu mengembalikan
        transform TERBARU, jadi penerbit TF yang berhenti akan menyerahkan
        sampel identik tiap tick. Merekamnya membuat TF beku terlihat seperti
        drone yang melayang sempurna -- kebohongan yang mahal untuk data yang
        dipakai menilai pergerakan.
        """
        try:
            tf = self._tf_buffer.lookup_transform(
                self._map_frame, self._base_frame, Time())
        except TransformException:
            return

        stamp_ns = (int(tf.header.stamp.sec) * 1_000_000_000
                    + int(tf.header.stamp.nanosec))
        if stamp_ns == self._last_traj_ns:
            return
        self._last_traj_ns = stamp_ns

        row = self._base()
        row['stamp_ns'] = stamp_ns
        row['frame_id'] = self._map_frame
        row['x'] = tf.transform.translation.x
        row['y'] = tf.transform.translation.y
        row['z'] = tf.transform.translation.z
        row['yaw_deg'] = round(
            yaw_deg_from_quaternion(tf.transform.rotation), 4)
        self._sinks['trajectory'].write(row)

    @staticmethod
    def _flatten(d):
        """
        Datarkan nilai bersarang jadi satu tingkat agar muat di CSV.

        Dict bersarang menjadi `induk_anak`; list menjadi JSON. Hanya berlaku
        satu tingkat, karena itulah kedalaman yang benar-benar dipakai
        muatannya (mis. `state_durations_s`).
        """
        out = {}
        for k, v in d.items():
            if isinstance(v, dict):
                for k2, v2 in v.items():
                    out['%s_%s' % (k, k2)] = v2
            elif isinstance(v, (list, tuple)):
                out[k] = json.dumps(v)
            elif isinstance(v, bool):
                out[k] = int(v)
            else:
                out[k] = v
        return out

    def destroy_node(self):
        """Laporkan jumlah baris tiap berkas, lalu tutup semuanya."""
        for name, s in self._sinks.items():
            self.get_logger().info('%-12s %5d baris -> %s'
                                   % (name, s.rows, s.path.name))
            s.close()
        super().destroy_node()


def main():
    """Titik masuk executable `logger_stats`."""
    rclpy.init()
    node = None
    try:
        node = StatsLoggerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
