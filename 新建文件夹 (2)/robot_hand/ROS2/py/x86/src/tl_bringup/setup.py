from setuptools import setup
from glob import glob
import os

package_name = "tl_bringup"

data_files = [
    ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    ("share/" + package_name, ["package.xml"]),
]

# 安装 launch 文件
for p in glob("launch/*.launch.py"):
    data_files.append(("share/" + package_name + "/launch", [p]))

# 安装 doc 文件
for p in glob("doc/*"):
    data_files.append(("share/" + package_name + "/doc", [p]))

setup(
    name=package_name,
    version="0.0.0",
    packages=[],
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rm",
    maintainer_email="tl@todo.todo",
    description="天链机器人 tl_bringup 功能包，实现多个launch文件同时运行",
    license="TODO: License declaration",
    tests_require=["pytest"],
)
