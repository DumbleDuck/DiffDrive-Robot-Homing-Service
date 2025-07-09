from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'skid_steer_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][y]'))),
        (os.path.join('share', package_name, 'URDF'), glob(os.path.join('URDF', '*.urdf'))),
        (os.path.join('share', package_name, 'URDF', 'assets'), glob(os.path.join('URDF', 'assets', '*.*'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.world'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'srv'), glob(os.path.join('srv', '*.srv'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mutant',
    maintainer_email='mutant@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'homing_server = skid_steer_robot.homing_server:main',
        ],
    },
)
