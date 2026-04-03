from setuptools import find_packages, setup

package_name = 'yolo_realsense'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/yolo_realsense', ['yolo_realsense/yolosegv1.pt']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lucky',
    maintainer_email='lucky@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'detector = yolo_realsense.yolo_distance_node:main',
            'yolo_to_moveit = yolo_realsense.yolo_to_moveit_node:main',
        ],
    },
)
