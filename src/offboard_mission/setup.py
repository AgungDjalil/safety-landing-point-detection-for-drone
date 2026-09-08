from setuptools import find_packages, setup

package_name = 'offboard_mission'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alphaone',
    maintainer_email='agungfirmansyahdjalil@gmail.com',
    description='Autonomous PX4 offboard mission: arm, take off, fly to a waypoint, scan for a safe landing point, and land on it.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'waypoint_node = offboard_mission.waypoint_node:main',
        ],
    },
)
