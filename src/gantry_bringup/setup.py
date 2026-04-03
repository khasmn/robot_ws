from setuptools import find_packages, setup

package_name = 'gantry_bringup'

import glob

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*.py')),
        ('share/' + package_name, ['gantry.urdf.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lucky',
    maintainer_email='67011698@kmitl.ac.th',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'bringup = gantry_bringup.bringup:main'
        ],
    },
)
