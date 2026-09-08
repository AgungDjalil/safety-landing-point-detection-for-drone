from setuptools import find_packages, setup

package_name = 'px4_offboard_lib'

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
    description='Pure helpers for PX4 offboard control. Library only, no nodes.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    # No console_scripts: this package ships a library, not an executable.
    entry_points={},
)
