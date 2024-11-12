from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'robot_lidar'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='korol',
    maintainer_email='korolnastasia@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'robot_lidar = robot_lidar.robot_lidar:main'
        ],
    },
)<maintainer email="korolnastasia@gmail.com">korol</maintainer>
