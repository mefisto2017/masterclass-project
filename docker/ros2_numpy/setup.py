from setuptools import setup, find_packages

setup(
    name='ros2_numpy',
    version='0.1.0',    
    description='description',
    url='https://github.com/Box-Robotics/ros2_numpy',
    packages=find_packages(),
    install_requires=[
        'numpy',
    ],
)

