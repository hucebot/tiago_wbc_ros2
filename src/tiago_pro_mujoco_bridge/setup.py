from setuptools import setup
import os
from glob import glob

package_name = 'tiago_pro_mujoco_bridge'

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
    maintainer='Yiannis Loizou',
    maintainer_email='yiannisloizou@gmail.com',
    description='MuJoCo-based simulated hardware bridge for Tiago Pro',
    license='TODO',
    entry_points={
        'console_scripts': [
            'mujoco_bridge_node = tiago_pro_mujoco_bridge.mujoco_bridge_node:main',
        ],
    },
)
