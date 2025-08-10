from setuptools import setup

package_name = 'extrinsic_calibration'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Team766',
    maintainer_email='team@team766.com',
    description='ROS2 package for extrinsic camera calibration data collection',
    license='MIT',
    entry_points={
        'console_scripts': [
            'data_collector = extrinsic_calibration.data_collector:main',
            'solver = extrinsic_calibration.solver:main',
        ],
    },
)