from setuptools import setup, find_packages

setup(
    name='detection_tools',
    version='0.1.0',
    packages=find_packages(),
    install_requires=[
        'ultralytics>=8.0.0',
        'onnx',
        'onnxsim',
        'numpy<2',
    ],
    entry_points={
        'console_scripts': [
            'convert_to_onnx=detection_tools.convert_to_onnx:main',
        ],
    },
    python_requires='>=3.8',
    author='cpadwick',
    author_email='cgpadwick@gmail.com',
    description='Utilities for YOLO model conversion and detection tasks',
)
