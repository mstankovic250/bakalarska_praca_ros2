from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pokus'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('pokus/*.npz')),
    ],
    install_requires=['setuptools', 'numpy', 'tf2_ros', 'tf2_geometry_msgs',],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pokus_node = pokus.pokus_node:main',
            'select_points = pokus.select_points:main',
            'aruco_navigator = pokus.aruco_navigator:main',
        ],
    },
)
