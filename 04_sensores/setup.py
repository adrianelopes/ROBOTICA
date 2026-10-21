import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'sensores'


def package_files(directory):
    return [
        (os.path.join('share', package_name, path),
         [os.path.join(path, f) for f in files])
        for path, _, files in os.walk(directory) if files
    ]


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    ] + package_files('models'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Fernanda, Renan, Adriane',
    maintainer_email='todos@todo.todo',
    description='Sensores (IMU, GPS, câmera RGB e lidar 2D) no robô my_robot',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'cmd_vel_relay = sensores.cmd_vel_relay:main',
            'imu_orientacao = sensores.imu_orientacao:main',
            'gps_xyz = sensores.gps_xyz:main',
            'scan_para_nuvem = sensores.scan_para_nuvem:main',
            'estimador_pose = sensores.estimador_pose:main',
            'trajetoria_8 = sensores.trajetoria_8:main',
        ],
    },
)
