from setuptools import find_packages, setup

package_name = 'path_trail'

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
    maintainer='mobi',
    maintainer_email='agungfirmansyahdjalil@gmail.com',
    description='Publishes the drone historical trajectory as nav_msgs/Path for RViz2.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "path_trail_node = path_trail.path_trail_node:main"
        ],
    },
)
