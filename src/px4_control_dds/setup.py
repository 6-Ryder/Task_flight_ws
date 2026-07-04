from setuptools import find_packages, setup
import os; from glob import glob

package_name = 'px4_control_dds'

setup(
    name=package_name, version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='llc', maintainer_email='llc@todo.todo',
    description='Mission controller via micro-XRCE-DDS',
    license='MIT',
    entry_points={'console_scripts': [
        'mission_control_dds_node = px4_control_dds.mission_control_node:main',
    ]},
)
