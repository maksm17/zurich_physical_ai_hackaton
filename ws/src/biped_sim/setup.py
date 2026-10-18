from glob import glob

from setuptools import setup

package_name = 'biped_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maks',
    maintainer_email='maksmajdanski@gmail.com',
    description='MuJoCo simulation of the biped (drop-in for the real hardware)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'sim_node = biped_sim.sim_node:main',
        ],
    },
)
