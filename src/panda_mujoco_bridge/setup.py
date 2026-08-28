from setuptools import setup
import os
from glob import glob

package_name = 'panda_mujoco_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Constantinos Tsakonas',
    maintainer_email='iamtsac@gmail.com',
    description='MuJoCo-based simulated hardware bridge for the Franka Panda',
    license='TODO',
    entry_points={
        'console_scripts': [
            'panda_sim_node = panda_mujoco_bridge.panda_sim_node:main',
        ],
    },
)
