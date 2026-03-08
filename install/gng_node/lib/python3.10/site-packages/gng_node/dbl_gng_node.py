import numpy as np
import rclpy
import torch
import struct
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

from gng_node.dbl_gng import DBL_GNG

class DBLGNGNode(Node):
    def __init__(self):
        super().__init__("dbl_gng")

        self.gng = DBL_GNG(
            feature_number=3,
            max_nodes=850,
            alpha=0.5,
            beta=0.01,
            delta=0.5,
            rho=0.5,
            eps=1e-4,
            planarity_threshold=0.00001,
            min_cluster_size=20,
            max_normal_angle_deg=5.0,
            normal_axis=[1, 0, 0],
            node_normal_radius=3,
        )

        self.outlier_fields_ = [...]
        self.rgb_red_ = 0xFF0000

        self.latest_points: np.ndarray | None = None

        self.marker_pub = self.create_publisher(Marker,      "/graph_markers", 1)
        self.flat_pub   = self.create_publisher(PointCloud2, "/plane",   1)
        self.outlier_pub = self.create_publisher(PointCloud2, "/outlier", 1)  # ✅ Changed to PointCloud2

        self.create_subscription(
            PointCloud2,
            "/depth_camera/points",
            # "/zed/zed_node/point_cloud/cloud_registered",
            self._pointcloud_callback,
            5,
        )

        self.create_timer(0.01, self._process_gng)
        self.create_timer(0.1, self._process_plane)
        self.create_timer(0.1, self._process_outlier)

        self.get_logger().info(
            f"DBL-GNG node started | device: {self.gng.device} | "
            f"planarity_threshold: {self.gng.planarity_threshold}"
        )

    def _pointcloud_callback(self, msg: PointCloud2):
        points = list(
            point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        )

        if not points:
            return

        arr                = np.array(points)
        pts_np             = np.column_stack((arr['x'], arr['y'], arr['z'])).astype(np.float32)
        pts_t              = torch.from_numpy(pts_np).to(self.gng.device)
        mask               = torch.isfinite(pts_t).all(dim=1)
        self.latest_points = pts_t[mask].cpu().numpy()

    def _process_gng(self):
        if self.latest_points is None:
            return

        data = self.latest_points
        if len(data) < 3:
            return

        if not self.gng.is_initialized:
            self.gng.initialize(data, number_of_starting_points=10)
            if not self.gng.is_initialized:
                return
            self.get_logger().info(f"GNG initialized with {len(self.gng.W)} nodes")

        self.gng.step(data)

    def _process_plane(self):
        if self.latest_points is None:
            return

        data = self.latest_points
        if len(data) < 3:
            return

        # Publish topology graph
        W_np, C_np = self.gng.get_numpy()
        self._publish_graph(W_np, C_np)

        # Publish flat points
        perp_pts, _ = self.gng.get_perpendicular_points_numpy()
        self._publish_flat_points(perp_pts)

    def _process_outlier(self):
        if self.latest_points is None:
            return

        data = self.latest_points
        if len(data) < 3:
            return

        # Publish outlier points
        outlier_pts = self.gng.get_outlier_points_numpy()
        self._publish_outliers(outlier_pts)

    def _publish_flat_points(self, points: np.ndarray):
        header          = Header()
        header.stamp    = self.get_clock().now().to_msg()
        header.frame_id = "camera_link"
        pts_list        = points[:, :3].tolist() if len(points) > 0 else []
        self.flat_pub.publish(point_cloud2.create_cloud_xyz32(header, pts_list))

    def _publish_outliers(self, points: np.ndarray):
        if len(points) == 0:
            return

        # Vectorized filtering
        valid_mask = np.isfinite(points[:, :3]).all(axis=1)
        valid_points = points[valid_mask, :3]
        
        if len(valid_points) == 0:
            return
        
        # Create structured array
        pcd_data = np.empty(len(valid_points), dtype=[
            ('x', np.float32), ('y', np.float32), ('z', np.float32), ('rgb', np.uint32)
        ])
        
        pcd_data['x'] = valid_points[:, 0]
        pcd_data['y'] = valid_points[:, 1]
        pcd_data['z'] = valid_points[:, 2]
        pcd_data['rgb'] = 0xFF0000  # Merah
        
        msg = point_cloud2.create_cloud(
            Header(stamp=self.get_clock().now().to_msg(), frame_id="camera_link"),
            [
                point_cloud2.PointField(name='x', offset=0, datatype=point_cloud2.PointField.FLOAT32, count=1),
                point_cloud2.PointField(name='y', offset=4, datatype=point_cloud2.PointField.FLOAT32, count=1),
                point_cloud2.PointField(name='z', offset=8, datatype=point_cloud2.PointField.FLOAT32, count=1),
                point_cloud2.PointField(name='rgb', offset=12, datatype=point_cloud2.PointField.UINT32, count=1),
            ],
            pcd_data
        )

        self.outlier_pub.publish(msg)

    def _publish_graph(self, node_list: np.ndarray, edge_list: np.ndarray):
        stamp     = self.get_clock().now().to_msg()
        frame_id  = "camera_link"
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
    node = DBLGNGNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()