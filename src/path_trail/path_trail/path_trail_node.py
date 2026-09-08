#!/usr/bin/env python3
"""
Publishes the drone's historical trajectory as nav_msgs/Path for RViz2.

Subscribes to the TF tree (map -> base_link, published by drone_kinematic) and
samples the drone's ENU position at a fixed rate, accumulating the samples into
a nav_msgs/Path published on /drone_path, which RViz2 draws as a continuous
line showing where the drone has been.

`use_sim_time:=true` MUST be passed -- the launch file does it, and running
this node by hand without it collects nothing. TF is stamped on the gz sim
clock; a buffer queried on the wall clock finds every transform either far in
the future or far in the past, and the lookup fails silently on every tick.
Nothing crashes, no warning is printed, and /drone_path simply stays empty.

(The docstring here used to claim the parameter defaults to True. It does not
-- rclpy defaults it to False, and nothing in this node declares otherwise.)

This node is for LOOKING at. The trajectory as DATA -- for plotting after the
flight -- is recorded separately by logger_stats into <run>_trajectory.csv,
deliberately not here: disabling the picture with path_trail:=false must not
also delete the numbers.
"""

from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from tf2_ros import Buffer, TransformException, TransformListener


def stamp_to_ns(stamp) -> int:
    """Collapse a builtin_interfaces/Time into one integer of nanoseconds."""
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class PathTrailNode(Node):
    """Accumulate TF samples into a growing nav_msgs/Path for RViz2."""

    def __init__(self) -> None:
        """Set up the TF listener and the sampling timer."""
        super().__init__("path_trail")

        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("sample_hz", 10.0)
        self.declare_parameter("topic", "/drone_path")

        # The whole Path is republished on every tick, so its length is not a
        # memory question -- it is a bandwidth question. Unbounded, ten minutes
        # of flight becomes 6000 poses resent ten times a second, roughly
        # 3 MB/s through DDS. That load lands on the same machine whose
        # computation_time we are trying to measure, so it would quietly
        # corrupt the numbers this project exists to collect.
        self.declare_parameter("max_poses", 3000)   # 5 minutes at 10 Hz

        self._map_frame = self.get_parameter("map_frame").value
        self._base_frame = self.get_parameter("base_frame").value
        self._topic = self.get_parameter("topic").value
        self._max_poses = int(self.get_parameter("max_poses").value)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._path = Path()
        self._path.header.frame_id = self._map_frame

        # Stamp of the last transform actually appended, to reject repeats.
        self._last_stamp_ns = None

        self._pub = self.create_publisher(Path, self._topic, 10)

        period = 1.0 / float(self.get_parameter("sample_hz").value)
        self._timer = self.create_timer(period, self._on_tick)

    def append_transform(self, tf) -> bool:
        """
        Add one TF sample to the trail. False if it was rejected.

        Split out from the timer so the trail's two rules can be tested
        without a live TF tree.
        """
        stamp_ns = stamp_to_ns(tf.header.stamp)

        # lookup_transform(..., Time()) always returns the LATEST transform,
        # so a stalled drone_kinematic hands back the same sample every tick.
        # Appending those would pile up hundreds of identical poses and make a
        # frozen TF tree look like a drone hovering perfectly still.
        if self._last_stamp_ns is not None and stamp_ns == self._last_stamp_ns:
            return False
        self._last_stamp_ns = stamp_ns

        pose = PoseStamped()
        pose.header.frame_id = self._map_frame
        pose.header.stamp = tf.header.stamp
        pose.pose.position.x = tf.transform.translation.x
        pose.pose.position.y = tf.transform.translation.y
        pose.pose.position.z = tf.transform.translation.z
        pose.pose.orientation = tf.transform.rotation

        self._path.poses.append(pose)
        if self._max_poses > 0:
            del self._path.poses[:-self._max_poses]

        # .to_msg() is the whole point of this line. get_clock().now() returns
        # an rclpy.time.Time -- the Python class used for time arithmetic --
        # while header.stamp demands a builtin_interfaces.msg.Time, the ROS
        # message with sec and nanosec fields. They share a name and a meaning
        # but not a type, and without the conversion this node dies on its
        # first tick with "The 'stamp' field must be a sub message of type
        # 'Time'". It did exactly that in flight.
        self._path.header.stamp = self.get_clock().now().to_msg()
        return True

    def _on_tick(self) -> None:
        try:
            tf = self._tf_buffer.lookup_transform(
                self._map_frame, self._base_frame, Time())
        except TransformException:
            return

        if self.append_transform(tf):
            self._pub.publish(self._path)


def main(args=None) -> None:
    """Entry point for the `path_trail_node` executable."""
    import rclpy
    from rclpy.executors import ExternalShutdownException

    rclpy.init(args=args)
    node = None
    try:
        node = PathTrailNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl+C and SIGTERM are how this node is meant to end. Letting them
        # escape prints a traceback that reads like a crash and buries the
        # real errors above it in the launch log.
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
