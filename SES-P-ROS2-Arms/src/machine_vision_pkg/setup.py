import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'machine_vision_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.xml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'assets'), glob('assets/*.glb')),
        (os.path.join('share', package_name, 'models'), glob('models/*.pt') + glob('models/*.pth')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alankrit',
    maintainer_email='agupta3129@gatech.edu',
    description='Machine-vision-driven control of the Lynxmotion pro_arm using MoveIt 2.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'move_to_a_point = machine_vision_pkg.move_to_a_point:main',
            'pick_and_place_example = machine_vision_pkg.pick_and_place_example:main',            
            'realsense_rgbd_viewer = machine_vision_pkg.realsense_rgbd_viewer:main',
            'two_stage_live_classifier = machine_vision_pkg.two_stage_live_classifier:main',
            'reprojection_3D = machine_vision_pkg.reprojection_3D:main',
            'classify_pick_and_place = machine_vision_pkg.classify_pick_and_place:main',
        ],
    },
)
