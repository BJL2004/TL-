from setuptools import setup
from glob import glob
import os

package_name = 'tl_bringup'

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

# install all launch files
for p in glob('launch/*.launch.py'):
    data_files.append(('share/' + package_name + '/launch', [p]))

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rm',
    maintainer_email='tl@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={},
)
