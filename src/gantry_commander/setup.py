from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gantry_commander'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
        (os.path.join('share', package_name), glob('*.urdf.xml')),
        (os.path.join('share', package_name), glob('*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='parallels',
    maintainer_email='kha150549@gmail.com',
    description='Gantry control package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_bridge_cli = gantry_commander.serial_bridge_cli:main',
            'go_to_point = gantry_commander.go_to_point:main',
            'serial_bridge = gantry_commander.serial_bridge:main',
        ],
    },
)