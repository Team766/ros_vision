from setuptools import find_packages, setup
from glob import glob
import os


package_name = 'ros_vision_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob(os.path.join('launch', '*.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cpadwick',
    maintainer_email='cgpadwick@gmail.com',
    description='ROS2 Vision Launch Utilities',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
