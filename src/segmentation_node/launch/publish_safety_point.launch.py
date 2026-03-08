# file: segmentation_pipeline.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Cylinder crop
        Node(
            package='segmentation_node',
            executable='cylinder_crop',
            name='cylinder_crop',
            output='screen'
        ),

        # Plane segmentation RANSAC
        Node(
            package='segmentation_node',
            executable='plane_segmentation_ransac',
            name='plane_segmentation_ransac',
            output='screen'
        ),

        # (opsional) tambahkan node lain di sini
        # Node(
        #     package='segmentation_node',
        #     executable='plane_segmentation_gng',
        #     name='plane_segmentation_gng',
        #     output='screen'
        # ),
    ])
