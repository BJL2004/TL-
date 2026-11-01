from setuptools import setup
from glob import glob
import os

package_name = "tl_teleop_f710"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        (
            os.path.join("share", "ament_index", "resource_index", "packages"),
            [os.path.join("resource", package_name)],
        ),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "udev"), glob("udev/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="example@mail.com",
    description="天链机械臂 F710 手柄遥操作功能包",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "tl_teleop_f710_node = tl_teleop_f710.tl_teleop_f710_node:main",
            "tl_teleop_f710_sim_bridge = tl_teleop_f710.tl_teleop_f710_sim_bridge:main",
        ],
    },
)
