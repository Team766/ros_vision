from setuptools import find_packages, setup

package_name = 'checkerboard_camera_calibration'

setup(
    name=package_name,
    version='0.0.0',
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
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [ 'checkerboard_camera_calibrator = checkerboard_camera_calibration.checkerboard_camera_calibrator:main',
        ],
    },
)
