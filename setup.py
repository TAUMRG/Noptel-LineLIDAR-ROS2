from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'Noptel_LineLIDAR_ROS2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]), ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'linelidarclass'), glob('linelidarclass/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mm',
    maintainer_email='mm@todo.todo',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'll_ros2 = Noptel_LineLIDAR_ROS2.ll_ros2:main'
        ],
    },
)
