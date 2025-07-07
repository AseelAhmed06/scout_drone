from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'drone_scout'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'scripts'), glob(os.path.join('scripts', 'prepare_data_folder.py'))),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aseel',
    maintainer_email='moinaseel17@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'geotag = drone_scout.geotag:main',
            'cam_test = drone_scout.cam_test:main',
            'cam_csi = drone_scout.cam_csi:main',
            'cam_gazebo = drone_scout.cam_gazebo:main',
            'drone_status = drone_scout.drone_status:main',
            'waypoint_generate = drone_scout.waypoint_generate:main',
            'inference = drone_scout.inference:main',
            'mission_commander = drone_scout.mission_commander:main',
            'demo = drone_scout.demo:main',
            'cam_usb = drone_scout.cam_usb:main',
        ],
    },
)
