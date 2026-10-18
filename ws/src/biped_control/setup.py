from glob import glob

from setuptools import setup

package_name = 'biped_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/trajectories', glob('trajectories/*.csv')),
        ('share/' + package_name + '/policies', glob('policies/*.npz')),
        ('share/' + package_name + '/udev', glob('udev/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maks',
    maintainer_email='maksmajdanski@gmail.com',
    description='Biped bringup, gait generator, MPU6050 driver and ST3215 tools',
    license='MIT',
    entry_points={
        'console_scripts': [
            'gait_node = biped_control.gait_node:main',
            'mpu6050_node = biped_control.mpu6050_node:main',
            'st3215_tool = biped_control.st3215_tool:main',
            'rl_policy_node = biped_control.rl_policy_node:main',
            'nav_fsm = biped_control.nav_fsm:main',
        ],
    },
)
