from setuptools import find_packages, setup

package_name = 'bag_utils'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cpadwick',
    maintainer_email='cgpadwick@gmail.com',
    description='Utilities for working with ROS bags',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'extract_images = bag_utils.extract_images:main',
        ],
    },
)
