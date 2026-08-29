from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'axioma_perception'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mario David Alvarez Vallejo',
    maintainer_email='ing.marioalvarezvallejo@gmail.com',
    description=('Axioma attitude-aware localization: EKF fusing a chassis IMU '
                 'with wheel or ZED 2i visual-inertial odometry'),
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [],
    },
)
