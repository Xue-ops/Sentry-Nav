from glob import glob
import os

from setuptools import setup


package_name = 'tf_trajectory_recorder'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            [os.path.join('resource', package_name)]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xli',
    maintainer_email='xueli2938229644@163.com',
    description='Record a TF frame trajectory and publish it as a nav_msgs/Path.',
    license='TODO',
    entry_points={
        'console_scripts': [
            'tf_trajectory_publisher = tf_trajectory_recorder.tf_trajectory_publisher:main',
        ],
    },
)
