from setuptools import find_packages, setup

package_name = 'keyboard_offboard_control'

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
    description='Keyboard-driven PX4 offboard velocity control for the safety-landing-point SITL drone.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'keyboard_offboard_node = keyboard_offboard_control.keyboard_offboard_node:main',
        ],
    },
)
