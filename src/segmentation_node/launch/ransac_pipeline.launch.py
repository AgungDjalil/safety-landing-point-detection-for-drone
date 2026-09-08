# Pipeline segmentasi RANSAC berdiri sendiri: plane_segmentation_ransac saja.
#
# BUKAN jalur pendaratan yang dipakai sekarang. Jalur itu adalah
# dbl_gng_cpu_node -> landing_circle, dan sejak offboard_mission ada, node
# misi (waypoint_node) yang menyalakannya sendiri saat drone tiba di
# waypoint -- tidak ada launch file untuk itu.
#
# Berkas ini dulu bernama publish_safety_point.launch.py, nama yang
# menjanjikan safety_point padahal tidak satu pun node di sini
# menerbitkannya.
#
# RIWAYAT: rantai ini dulu berupa cylinder_crop -> plane_segmentation_ransac.
# cylinder_crop memotong silinder pada sumbu x-y, padahal di `camera_link`
# justru x-lah sumbu KEDALAMAN -- /depth_camera/points membentang
# x 0,21..14,81 m (median 12,51) sementara bidang tanahnya ada di y-z, dan
# karena itulah landing_circle dikonfigurasi plane_axes=yz. Rantai lama itu
# karena itu membuang tanahnya: 19.200 titik masuk, 4 keluar (diuji ulang
# dengan awan sintetis: 14.641 masuk, 0 keluar), lalu
# plane_segmentation_ransac berhenti menerbitkan apa pun. Node cylinder_crop
# sudah DIHAPUS dari paket ini.
#
# Parameter di bawah disamakan dengan depth_bridge_launch.py (perception:=ransac)
# supaya RANSAC menerima awan yang sama persis dengan yang diterima GNG:
# PassThrough dimatikan (z +/-1000, sebab z adalah sumbu lateral di sini) dan
# leaf_size 0.15 sama dengan voxel_leaf GNG.
#
# use_sim_time=True mengharuskan ADA penerbit /clock (jembatan gz). Tanpa itu
# jam simulasi berhenti di 0 dan kolom latency_ms pada /segmentation_stats
# keluar sebagai angka negatif raksasa, bukan null. Jalankan bersama Gazebo.
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Plane segmentation RANSAC
        #
        # input_topic WAJIB dioper di sini. Default node-nya adalah
        # /zed/zed_node/point_cloud/cloud_registered -- warisan dari kamera ZED
        # asli, dan topik itu tidak pernah ada di simulasi Gazebo. Tanpa baris
        # ini node tetap hidup tapi bisu selamanya.
        Node(
            package='segmentation_node',
            executable='plane_segmentation_ransac',
            name='plane_segmentation_ransac',
            output='screen',
            parameters=[{
                'input_topic': '/depth_camera/points',
                'z_min': -1000.0,
                'z_max': 1000.0,
                'leaf_size': 0.15,
                'use_sim_time': True,
            }]
        ),
    ])
