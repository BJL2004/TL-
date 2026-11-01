from setuptools import setup
from glob import glob
import os
import shutil

package_name = "tl_driver"

# 将 x86 版 TL 库复制到 src/tl_driver/lib/ 供 package_data 打包
pkg_lib_dir = os.path.join("tl_driver", "lib")
os.makedirs(pkg_lib_dir, exist_ok=True)

for f in ("_tl_host.so", "tl_interface.py"):
    src = os.path.join("lib", "x86", f)
    dst = os.path.join(pkg_lib_dir, f)
    if os.path.exists(src):
        shutil.copy2(src, dst)

# data files to install so that ros2 can find launch/config and the swig lib
data_files = [
    ("share/ament_index/resource_index/packages", ["resource/tl_driver"]),
    ("share/" + package_name, ["package.xml"]),
]

# include launch and config files if present
for p in glob("launch/*.py"):
    data_files.append(("share/" + package_name + "/launch", [p]))
for p in glob("config/*"):
    data_files.append(("share/" + package_name + "/config", [p]))

setup(
    name=package_name,
    version="1.0.0",
    packages=["tl_driver", "tl_driver.lib"],
    package_dir={"": "."},
    package_data={"tl_driver.lib": ["_tl_host.*", "tl_interface.py"]},
    data_files=data_files,
    include_package_data=True,
    zip_safe=False,
    install_requires=["setuptools"],
    maintainer="Your Name",
    maintainer_email="your_email@example.com",
    description="ROS2 driver for TL series robotic arm (Python)",
    license="Apache-2.0",
    # cmdclass={'build_py': build_py},
    entry_points={
        "console_scripts": [
            "tl_driver_node = tl_driver.tl_driver_node:main",
        ],
    },
)
