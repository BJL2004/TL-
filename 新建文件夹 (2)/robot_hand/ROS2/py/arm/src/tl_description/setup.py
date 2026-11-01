from setuptools import setup
from glob import glob
import os

package_name = 'tl_description'

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

# 安装 launch 文件
for p in glob('launch/*.launch.py'):
    data_files.append(('share/' + package_name + '/launch', [p]))

# 安装 config 文件
for p in glob('config/*.yaml'):
    data_files.append(('share/' + package_name + '/config', [p]))

# 安装 urdf 文件
for p in glob('urdf/*'):
    if os.path.isfile(p):
        data_files.append(('share/' + package_name + '/urdf', [p]))

# 安装 rviz 文件
for p in glob('rviz/*.rviz'):
    data_files.append(('share/' + package_name + '/rviz', [p]))

# 安装 doc 文件
for p in glob('doc/*'):
    if os.path.isfile(p):
        data_files.append(('share/' + package_name + '/doc', [p]))

# 递归安装 meshes 文件，保持目录结构
for root, dirs, files in os.walk('meshes'):
    if files:
        install_dir = os.path.join('share', package_name, root)
        file_paths = [os.path.join(root, f) for f in files]
        data_files.append((install_dir, file_paths))

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='my_email@example.com',
    description='天链机器人 tl_description 功能包，显示机器人模型和TF变换',
    license='TODO: License declaration',
    tests_require=['pytest'],
)
