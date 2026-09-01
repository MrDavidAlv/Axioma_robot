from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'axioma_gazebo'


def get_data_files_recursive(source_dir, install_prefix):
    """Generate data_files entries for a directory tree."""
    data_files = []
    for dirpath, _, filenames in os.walk(source_dir):
        if filenames:
            install_dir = os.path.join(
                install_prefix, os.path.relpath(dirpath, source_dir)
            )
            files = [os.path.join(dirpath, f) for f in filenames]
            data_files.append((install_dir, files))
    return data_files


data_files = [
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),
]
data_files += get_data_files_recursive(
    'models', os.path.join('share', package_name, 'models'))
data_files += get_data_files_recursive(
    'worlds', os.path.join('share', package_name, 'worlds'))

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mario David Alvarez Vallejo',
    maintainer_email='ing.marioalvarezvallejo@gmail.com',
    description='Axioma Gz Sim simulation, bridge and spawn for ROS2 Humble',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'odom_to_tf = axioma_gazebo.odom_to_tf:main',
        ],
    },
)
