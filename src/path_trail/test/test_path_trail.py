"""
Unit test untuk PathTrailNode.

Berkas ini ada karena node-nya mati di udara pada tick pertama:

    self._path.header.stamp = self.get_clock().now()
    AssertionError: The 'stamp' field must be a sub message of type 'Time'

`get_clock().now()` mengembalikan `rclpy.time.Time`, sedangkan `header.stamp`
menuntut `builtin_interfaces.msg.Time`. Keduanya bernama sama dan mewakili hal
yang sama, tapi bukan tipe yang sama. Tidak ada satu tes pun yang menjaga itu,
jadi ketahuannya baru saat launch berjalan.

Yang diuji di sini adalah kontrak jejaknya, bukan TF-nya: transform dibuat
tangan, sehingga tidak perlu ada drone yang terbang.
"""

import pytest
import rclpy

from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import TransformStamped

from path_trail.path_trail_node import PathTrailNode, stamp_to_ns


@pytest.fixture
def node():
    """PathTrailNode hidup di atas konteks rclpy sementara."""
    rclpy.init()
    n = PathTrailNode()
    yield n
    n.destroy_node()
    rclpy.shutdown()


def tf_at(sec, nanosec=0, x=0.0, y=0.0, z=0.0):
    """Satu TransformStamped buatan, tanpa TF tree."""
    t = TransformStamped()
    t.header.stamp = TimeMsg(sec=sec, nanosec=nanosec)
    t.header.frame_id = 'map'
    t.child_frame_id = 'base_link'
    t.transform.translation.x = float(x)
    t.transform.translation.y = float(y)
    t.transform.translation.z = float(z)
    t.transform.rotation.w = 1.0
    return t


# ── regresi crash ───────────────────────────────────────────────────────────

def test_path_stamp_is_a_ros_message_not_an_rclpy_time(node):
    """
    Stempel Path harus bertipe pesan ROS.

    Inilah crash-nya: `rclpy.time.Time` lolos tanpa keluhan sampai
    detik pertama runtime, lalu mematikan node.
    """
    node.append_transform(tf_at(10))
    assert isinstance(node._path.header.stamp, TimeMsg)


def test_appending_does_not_raise(node):
    """Tick pertama dulu melempar AssertionError; sekarang tidak boleh."""
    assert node.append_transform(tf_at(1)) is True


# ── isi pose ────────────────────────────────────────────────────────────────

def test_pose_carries_the_transform_position_and_frame(node):
    node.append_transform(tf_at(3, x=1.5, y=-2.25, z=11.0))

    pose = node._path.poses[-1]
    assert (pose.pose.position.x, pose.pose.position.y,
            pose.pose.position.z) == (1.5, -2.25, 11.0)
    assert pose.header.frame_id == 'map'
    assert node._path.header.frame_id == 'map'


def test_pose_keeps_the_transform_stamp_not_the_clock(node):
    """
    Tiap pose distempel waktu TRANSFORM-nya, bukan waktu tick.

    Kalau dipakai waktu tick, lintasannya tidak bisa lagi disejajarkan dengan
    CSV lain yang memakai stempel awan titik.
    """
    node.append_transform(tf_at(7, nanosec=250_000_000))
    assert stamp_to_ns(node._path.poses[-1].header.stamp) == 7_250_000_000


# ── penolakan sampel kembar ─────────────────────────────────────────────────

def test_identical_stamp_is_rejected(node):
    """
    TF yang membeku tidak boleh terlihat seperti drone yang melayang diam.

    lookup_transform selalu mengembalikan transform TERBARU, jadi
    drone_kinematic yang berhenti menyerahkan sampel yang sama tiap tick.
    """
    assert node.append_transform(tf_at(5, x=1.0)) is True
    assert node.append_transform(tf_at(5, x=9.0)) is False
    assert len(node._path.poses) == 1


def test_a_new_stamp_is_accepted_again(node):
    node.append_transform(tf_at(5))
    node.append_transform(tf_at(5))
    assert node.append_transform(tf_at(6)) is True
    assert len(node._path.poses) == 2


# ── batas panjang ───────────────────────────────────────────────────────────

def test_trail_is_capped_and_drops_the_oldest(node):
    """
    Seluruh Path diterbitkan ulang tiap tick, jadi panjangnya soal bandwidth.

    Tanpa batas, sepuluh menit terbang jadi 6000 pose yang dikirim ulang
    sepuluh kali per detik -- beban yang jatuh ke mesin yang sedang diukur
    waktu komputasinya.
    """
    node._max_poses = 3
    for sec in range(1, 6):
        node.append_transform(tf_at(sec, x=float(sec)))

    assert len(node._path.poses) == 3
    xs = [p.pose.position.x for p in node._path.poses]
    assert xs == [3.0, 4.0, 5.0], 'yang tertua yang dibuang, bukan yang terbaru'


def test_zero_cap_means_unbounded(node):
    node._max_poses = 0
    for sec in range(1, 30):
        node.append_transform(tf_at(sec))
    assert len(node._path.poses) == 29


# ── helper ──────────────────────────────────────────────────────────────────

def test_stamp_to_ns_combines_both_fields():
    assert stamp_to_ns(TimeMsg(sec=2, nanosec=500_000_000)) == 2_500_000_000
