from setuptools import setup
from glob import glob
import os

package_name = "tl_gazebo"

data_files = [
    ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    ("share/" + package_name, ["package.xml"]),
]

# 安装 launch 文件
for p in glob("launch/*.launch.py"):
    data_files.append(("share/" + package_name + "/launch", [p]))

# 安装 config 文件
for p in glob("config/*"):
    if os.path.isfile(p):
        data_files.append(("share/" + package_name + "/config", [p]))

setup(
    name=package_name,
    version="0.3.0",
    packages=[],
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="tl",
    maintainer_email="tl@robot.com",
    description="Gazebo simulation package for TL series robotic arm",
    license="BSD",
    tests_require=["pytest"],
)
