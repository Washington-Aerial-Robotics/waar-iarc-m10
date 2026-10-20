import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files:
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Include all config files:
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # Include all URDF files:
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='waar1',
    maintainer_email='waar1@todo.todo',
    description='SLAM configuration package',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'stereo_splitter = slam.stereo_splitter:main',
            'esp32_bridge = slam.esp32_bridge:main'
        ],
    },
)
