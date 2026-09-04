import os
from glob import glob
from setuptools import setup

package_name = 'asinus_sensors'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rinaldo Filho',
    maintainer_email='rinaldo.filho.095@ufrn.edu.br',
    description='Sensores do Asinus no Orange Pi 3B: ICM-20948 + GPS + fusao',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'icm20948_node = asinus_sensors.icm20948_node:main',
        ],
    },
)
