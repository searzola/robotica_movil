from setuptools import find_packages, setup
from glob import glob

package_name = 'proyecto_grupo_5_2024'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.xml')),
        ('share/' + package_name, glob('*.yaml')),
        ('share/' + package_name, glob('*.pgm')),
        ('share/' + package_name, glob('config/*.rviz')),
        ('share/' + package_name, glob('params/*.yaml')),
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
        ],
    },
)
