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
            "angular_pid_controller_node = lab_2.angular_pid_controller:main"
        ],
    },
)
