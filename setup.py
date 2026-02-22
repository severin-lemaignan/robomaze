from setuptools import setup
from glob import glob

package_name = 'robomaze'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/res', glob('res/*')),
        ('share/' + package_name + '/cfg', glob('cfg/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='Interactive 2D maze simulator for ROS2',
    entry_points={
        'console_scripts': [
            'simulator = robomaze.simulator_node:main',
            'keyboard_teleop = robomaze.keyboard_teleop:main',
        ],
    },
)
