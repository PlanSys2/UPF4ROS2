from glob import glob
import os

from setuptools import setup

package_name = 'upf4ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'pddl'), glob('tests/pddl/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fmrico',
    maintainer_email='fmrico@gmail.com',
    description='ROS 2 Support for UPF',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'upf4ros2_main = upf4ros2.upf4ros2_main:main'
        ],
    },
)
