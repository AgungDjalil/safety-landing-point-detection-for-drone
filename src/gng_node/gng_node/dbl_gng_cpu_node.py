import json
import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

from gng_node.dbl_gng_cpu import DBL_GNG_CPU

class DBLGNGCpuNode(Node):
    def __init__(self):
        super().__init__("dbl_gng_cpu")

        self.gng = DBL_GNG_CPU(
            feature_number=3,
            max_nodes=550,
            alpha=0.5,
            beta=0.03,
            delta=0.7,
            rho=0.7,
            eps=1e-4,
            planarity_threshold=0.0001,
            min_cluster_size=20,
            max_normal_angle_deg=10.0,
            normal_axis=[1, 0, 0],
            node_normal_radius=3,
        )

        self.latest_points: np.ndarray | None = None
        self._latest_header = Header(frame_id="camera_link")

        # ── metrik satu frame ─────────────────────────────────────────────────
        # Dulu jumlah titik valid hanya dihitung ulang tiap frame ke-5 lalu
        # nilai lamanya diterbitkan ulang di antaranya. Sebagai telemetri kasar
        # itu tidak apa-apa; sebagai data per frame untuk dibandingkan dengan
        # backend lain itu salah — empat dari lima baris membawa angka yang
        # bukan miliknya. Sekarang dihitung tiap frame: satu lintasan numpy
        # atas ~19k titik, tidak berarti dibanding langkah GNG-nya sendiri.
        self._valid_pts    = 0
        self._total_pts    = 0
        self._valid_pct    = 0.0
        self._downsampled  = 0
        self._stamp_ns     = 0

        self.comp_ms = 0.0

        # ── ROS parameters ──────────────────────────────────────────────────
        # pointcloud_topic: sumber PointCloud2.
        #   - Gazebo stack (gz_bridge_ros2): /depth_camera/points  (default)
        #   - ZED camera real             : /zed/zed_node/point_cloud/cloud_registered
        # use_sim_time dikelola otomatis oleh rclpy; set ke true dari launch
        # file agar get_clock().now() mengikuti /clock (gz sim time) dan stamp
        # output (/plane_cpu, /outlier_cpu, /graph_markers_cpu) match dengan TF.
        self.declare_parameter(
            "pointcloud_topic", "/depth_camera/points")
        pc_topic = self.get_parameter("pointcloud_topic").value

        # voxel_leaf: ukuran sel voxel (meter) untuk downsampling sebelum GNG.
        #
        # Ini yang menahan beban GNG: biaya batch_learning kira-kira linier
        # terhadap jumlah titik (pencarian BMU = N titik x max_nodes), jadi
        # leaf inilah tombol utama antara kerapatan dan laju.
        #
        # Nilai yang tepat TIDAK bisa ditentukan dari meja — kapasitas voxel
        # per meter persegi bergantung pada ketinggian terbang dan kemiringan
        # permukaan terhadap kamera. Sebagai acuan terukur: pada sensor 80x60,
        # leaf 0.1 m hanya membuang 1% titik (4790 -> 4742) karena kerapatan
        # mentahnya memang jauh di bawah kapasitas voxel. Sejak sensor
        # dinaikkan ke 160x120, leaf mulai benar-benar mengikat.
        #
        # Naikkan bila computation_time GNG terlalu besar; turunkan bila masih
        # ada anggaran waktu dan ingin cloud lebih rapat untuk landing_circle.
        self.declare_parameter("voxel_leaf", 0.15)
        self._voxel_leaf = float(self.get_parameter("voxel_leaf").value)

        self.marker_pub  = self.create_publisher(Marker,      "/graph_markers_cpu",      1)
        self.flat_pub    = self.create_publisher(PointCloud2, "/plane_cpu",              1)
        self.outlier_pub = self.create_publisher(PointCloud2, "/outlier_cpu",            1)
        self.stats_pub   = self.create_publisher(String,      "/segmentation_stats_cpu", 10)

        self.create_subscription(
            PointCloud2,
            pc_topic,
            self._pointcloud_callback,
            5,
        )

        self.create_timer(0.1, self._process_gng)
        self.create_timer(0.2, self._process_plane)
        self.create_timer(0.2, self._process_outlier)

        self.get_logger().info(
            f"DBL-GNG (CPU, multithreaded) node started | "
            f"workers: {self.gng.num_workers} | "
            f"pointcloud_topic: {pc_topic} | "
            f"use_sim_time: {self.get_parameter('use_sim_time').value} | "
            f"planarity_threshold: {self.gng.planarity_threshold}"
        )

    # ── helpers ───────────────────────────────────────────────────────────────

    def _update_valid_count(self, pts: np.ndarray):
        """Hitung titik valid (finite) untuk frame ini."""
        self._total_pts = len(pts)
        self._valid_pts = int(np.isfinite(pts).all(axis=1).sum())
        self._valid_pct = (
            self._valid_pts / self._total_pts * 100.0
            if self._total_pts > 0 else 0.0
        )

    def _publish_stats(self, comp_ms: float, plane_size: int, outlier_size: int):
        """Terbitkan metrik satu frame sebagai JSON.

        Muatannya JSON, bukan teks berformat, karena parser berbasis regex atas
        format bebas sudah patah senyap dua kali di proyek ini — logger CSV-nya
        berhenti menemukan koordinat kandidat tanpa satu pun pesan galat, dan
        berkasnya kosong berbulan-bulan. Menambah field ke JSON tidak akan
        pernah mematahkan pembacanya.

        Baris log konsol tidak ikut berubah; itu untuk manusia.
        """
        msg = String()
        msg.data = json.dumps({
            "source":             "gng",
            "stamp_ns":           self._stamp_ns,
            "computation_time_ms": round(comp_ms, 6),
            "latency_ms":         self._latency_ms(),
            "input_points":       self._total_pts,
            "valid_points":       self._valid_pts,
            "valid_percentage":   round(self._valid_pct, 6),
            "downsampled_points": self._downsampled,
            "plane_size":         int(plane_size),
            "outlier_size":       int(outlier_size),
            "voxel_leaf_m":       self._voxel_leaf,
        })
        self.stats_pub.publish(msg)

    def _latency_ms(self):
        """Usia awan saat hasilnya terbit: sekarang − stempel masukan.

        `computation_time` hanya mengukur bagian dalam callback; yang
        menentukan seberapa segar peta yang dipakai drone adalah angka ini.

        None bila belum ada stempel, atau bila use_sim_time mati. Yang kedua
        penting: stempel awan berasal dari jam simulasi Gazebo, jadi
        menguranginya dengan jam dinding menghasilkan angka besar yang
        konsisten — salah, tapi tidak tampak salah sepintas. Lebih baik tidak
        melaporkan apa pun daripada melaporkan itu.
        """
        if self._stamp_ns <= 0:
            return None
        if not self.get_parameter("use_sim_time").value:
            return None
        now_ns = self.get_clock().now().nanoseconds
        return round((now_ns - self._stamp_ns) / 1e6, 6)

    # ── helpers ───────────────────────────────────────────────────────────────

    @staticmethod
    def _voxel_downsample(pts: np.ndarray, leaf: float = 0.1) -> np.ndarray:
        """Voxel grid downsampling (first-occurrence per cell) — pure numpy."""
        if len(pts) == 0:
            return pts
        keys = np.floor(pts / leaf).astype(np.int64)
        kmin = keys.min(axis=0)
        keys = keys - kmin
        dims = keys.max(axis=0) + 1
        flat = keys[:, 0] * dims[1] * dims[2] + keys[:, 1] * dims[2] + keys[:, 2]
        _, idx = np.unique(flat, return_index=True)
        return pts[idx]

    # ── callbacks ─────────────────────────────────────────────────────────────

    def _pointcloud_callback(self, msg: PointCloud2):
        points = list(
            point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        )

        if not points:
            return

        arr    = np.array(points)
        pts_np = np.column_stack((arr['x'], arr['y'], arr['z'])).astype(np.float32)
        mask   = np.isfinite(pts_np).all(axis=1)
        pts_clean = pts_np[mask]

        # Voxel downsampling — kurangi ~110k pts → ~5-15k pts untuk GNG
        downsampled = self._voxel_downsample(pts_clean, leaf=self._voxel_leaf)
        self.latest_points = downsampled
        self._latest_header = msg.header
        self._downsampled = int(len(downsampled))

        # Stempel awan MASUKAN, digabung jadi satu bilangan bulat nanosekon.
        # Inilah kunci yang memungkinkan baris dari topik berbeda digabungkan:
        # waktu terima berbeda di tiap node dan bergeser oleh beban CPU,
        # sedangkan stempel ini diwarisi dari pesan kamera yang sama.
        self._stamp_ns = (int(msg.header.stamp.sec) * 1_000_000_000
                          + int(msg.header.stamp.nanosec))

        self._update_valid_count(pts_np)

    def _process_gng(self):
        data = self.latest_points
        if data is None or len(data) < 3:
            return

        t_start = time.perf_counter()

        if not self.gng.is_initialized:
            self.gng.initialize(data, number_of_starting_points=10)
            if not self.gng.is_initialized:
                return
            self.get_logger().info(f"GNG initialized with {len(self.gng.W)} nodes")

        self.gng.step(data)

        self.comp_ms = (time.perf_counter() - t_start) * 1000.0

    def _process_plane(self):
        data = self.latest_points
        if data is None or len(data) < 3:
            return

        W_np, C_np = self.gng.get_numpy()
        self._publish_graph(W_np, C_np)

        perp_pts, _ = self.gng.get_perpendicular_points_numpy()
        self._publish_flat_points(perp_pts)

    def _process_outlier(self):
        data = self.latest_points
        if data is None or len(data) < 3:
            return

        outlier_pts = self.gng.get_outlier_points_numpy()
        plane_pts, _ = self.gng.get_perpendicular_points_numpy()

        self._publish_outliers(outlier_pts)

        self._publish_stats(
            comp_ms=self.comp_ms,
            plane_size=len(plane_pts),
            outlier_size=len(outlier_pts),
        )
    # ── publishers ────────────────────────────────────────────────────────────

    def _publish_flat_points(self, points: np.ndarray):
        header = self._latest_header
        pts_list        = points[:, :3].tolist() if len(points) > 0 else []
        self.flat_pub.publish(point_cloud2.create_cloud_xyz32(header, pts_list))

    def _publish_outliers(self, points: np.ndarray):
        if len(points) == 0:
            return

        valid_mask   = np.isfinite(points[:, :3]).all(axis=1)
        valid_points = points[valid_mask, :3]

        if len(valid_points) == 0:
            return

        pcd_data = np.empty(len(valid_points), dtype=[
            ('x', np.float32), ('y', np.float32), ('z', np.float32), ('rgb', np.uint32)
        ])
        pcd_data['x']   = valid_points[:, 0]
        pcd_data['y']   = valid_points[:, 1]
        pcd_data['z']   = valid_points[:, 2]
        pcd_data['rgb'] = 0xFF0000  # merah

        msg = point_cloud2.create_cloud(
            self._latest_header,
            [
                point_cloud2.PointField(name='x',   offset=0,  datatype=point_cloud2.PointField.FLOAT32, count=1),
                point_cloud2.PointField(name='y',   offset=4,  datatype=point_cloud2.PointField.FLOAT32, count=1),
                point_cloud2.PointField(name='z',   offset=8,  datatype=point_cloud2.PointField.FLOAT32, count=1),
                point_cloud2.PointField(name='rgb', offset=12, datatype=point_cloud2.PointField.UINT32,  count=1),
            ],
            pcd_data
        )
        self.outlier_pub.publish(msg)

    def _publish_graph(self, node_list: np.ndarray, edge_list: np.ndarray):
        stamp     = self._latest_header.stamp
        frame_id  = self._latest_header.frame_id
        num_nodes = node_list.shape[0]

        node_marker                 = Marker()
        node_marker.header.frame_id = frame_id
        node_marker.header.stamp    = stamp
        node_marker.ns              = "nodes"
        node_marker.id              = 0
        node_marker.type            = Marker.SPHERE_LIST
        node_marker.action          = Marker.ADD
        node_marker.scale.x         = 0.05
        node_marker.scale.y         = 0.05
        node_marker.scale.z         = 0.05
        node_marker.color.r         = 0.1
        node_marker.color.g         = 0.8
        node_marker.color.b         = 0.1
        node_marker.color.a         = 1.0

        for n in node_list:
            if not np.isfinite(n[:3]).all():
                continue
            p = Point()
            p.x, p.y, p.z = float(n[0]), float(n[1]), float(n[2])
            node_marker.points.append(p)

        edge_marker                 = Marker()
        edge_marker.header.frame_id = frame_id
        edge_marker.header.stamp    = stamp
        edge_marker.ns              = "edges"
        edge_marker.id              = 1
        edge_marker.type            = Marker.LINE_LIST
        edge_marker.action          = Marker.ADD
        edge_marker.scale.x         = 0.01
        edge_marker.color.r         = 1.0
        edge_marker.color.g         = 1.0
        edge_marker.color.b         = 1.0
        edge_marker.color.a         = 0.6

        for e in edge_list:
            i, j = int(e[0]), int(e[1])
            if i >= num_nodes or j >= num_nodes:
                continue
            if not (np.isfinite(node_list[i, :3]).all()
                    and np.isfinite(node_list[j, :3]).all()):
                continue
            p1 = Point(); p1.x, p1.y, p1.z = map(float, node_list[i, :3])
            p2 = Point(); p2.x, p2.y, p2.z = map(float, node_list[j, :3])
            edge_marker.points.append(p1)
            edge_marker.points.append(p2)

        self.marker_pub.publish(node_marker)
        self.marker_pub.publish(edge_marker)


def main():
    rclpy.init()
    node = DBLGNGCpuNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
