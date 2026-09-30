import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'controle_trajetoria'


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
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ] + package_files('models'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Fernanda, Renan, Adriane',
    maintainer_email='todos@todo.todo',
    description='Controle de trajetória',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'cmd_vel_relay = controle_trajetoria.cmd_vel_relay:main',
            'seguidor_waypoints = controle_trajetoria.seguidor_waypoints:main',
        ],
    },
)
