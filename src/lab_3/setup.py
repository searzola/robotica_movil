from setuptools import find_packages, setup
from glob import glob

package_name = 'lab_3'

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
            "likelihood_map_node = lab_3.likelihood_map:main",
            "likelihood_test_node = lab_3.likelihood_test:main",
            "simple_particle_pose_changer_node = lab_3.simple_particle_pose_changer:main",
            "particles_manager_node = lab_3.particles_manager:main",
            "reactive_movement_node = lab_3.reactive_movement:main"
        ],
    },
)
