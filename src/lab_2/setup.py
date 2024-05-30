from setuptools import find_packages, setup
from glob import glob

package_name = 'lab_2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='searzola',
    maintainer_email='searzola@uc.cl',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "pose_loader_node = lab_2.pose_loader:main",
            "dead_reckoning_nav_real_odom_factor_ctrl_node = lab_2.dead_reckoning_nav_real_odom_factor_ctrl:main",
            "linear_pid_controller_node = lab_2.linear_pid_controller:main",
            "angular_pid_controller_node = lab_2.angular_pid_controller:main",
            "blue_watcher_node = lab_2.blue_watcher:main",
            "blue_stalker_node = lab_2.blue_stalker:main",
            "wall_distance_node = lab_2.wall_distance:main",
            "wall_controller_node = lab_2.wall_controller:main",
            "wall_actuation_node = lab_2.wall_actuation:main",
            "square_watcher_node = lab_2.square_watcher:main",
            "square_photos_node = lab_2.square_photos:main",
            "square_follower_node = lab_2.square_follower:main",
            "obstacle_detector2_node = lab_2.osbtacle_detector2:main"
        ],
    },
)
