from setuptools import find_packages, setup
from glob import glob

package_name = 'lab_1'

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
            "dead_reckoning_nav_node = lab_1.dead_reckoning_nav:main",
            "pose_loader_node = lab_1.pose_loader:main",
            "dead_reckoning_nav_obs_node = lab_1.dead_reckoning_nav_obs:main",
            "obstacle_detector_node = lab_1.obstacle_detector:main",
            "dead_reckoning_nav_real_odom_factor_node = lab_1.dead_reckoning_nav_real_odom_factor:main"
        ],
    },
)
