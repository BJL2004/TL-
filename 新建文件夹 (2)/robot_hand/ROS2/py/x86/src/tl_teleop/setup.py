from setuptools import setup
from setuptools.command.build_py import build_py
from glob import glob
import os
import subprocess
import sys

package_name = "tl_teleop"

# xrobotoolkit_sdk 子模块路径
_SDK_DIR = os.path.join(os.path.dirname(__file__), "depend", "XRoboToolkit-PC-Service-Pybind")


def _ensure_xrobotoolkit_sdk():
    """在构建 tl_teleop 前检查并安装 xrobotoolkit_sdk（Python 绑定库）。"""
    # 已安装则跳过
    try:
        import xrobotoolkit_sdk  # noqa: F401

        return
    except ImportError:
        pass

    # 检查子模块是否已初始化
    sdk_setup = os.path.join(_SDK_DIR, "setup.py")
    if not os.path.exists(sdk_setup):
        print("[tl_teleop] 警告: xrobotoolkit_sdk 子模块未初始化")
        print("[tl_teleop] 请运行: git submodule update --init")
        return

    # 检查原生库 libPXREARobotSDK.so 是否就绪，缺失则自动构建
    lib_x86 = os.path.join(_SDK_DIR, "lib", "libPXREARobotSDK.so")
    lib_arm = os.path.join(_SDK_DIR, "lib", "aarch64", "libPXREARobotSDK.so")
    if not os.path.exists(lib_x86) and not os.path.exists(lib_arm):
        # 根据平台选择构建脚本
        import platform

        machine = platform.machine()
        if machine in ("aarch64", "arm64"):
            setup_script = "setup_orin.sh"
        else:
            setup_script = "setup_ubuntu.sh"
        script_path = os.path.join(_SDK_DIR, setup_script)
        if not os.path.exists(script_path):
            print(f"[tl_teleop] 错误: 未找到 {setup_script}")
            print(f"[tl_teleop] 请手动安装 XRoboToolkit-PC-Service 后再构建")
            return
        print(f"[tl_teleop] 原生 SDK 缺失，正在自动构建（{setup_script}）...")
        subprocess.check_call(["bash", script_path], cwd=_SDK_DIR)

    # 安装 xrobotoolkit_sdk
    print("[tl_teleop] 正在安装 xrobotoolkit_sdk ...")
    subprocess.check_call(
        [sys.executable, "-m", "pip", "install", _SDK_DIR],
    )


class BuildPyWithSDK(build_py):
    """自定义 build_py：构建前自动安装 xrobotoolkit_sdk。"""

    def run(self):
        _ensure_xrobotoolkit_sdk()
        super().run()


data_files: list = [
    ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    ("share/" + package_name, ["package.xml"]),
]

# 安装 launch 文件（如果存在）
for p in glob("launch/*.launch.py"):
    data_files.append(("share/" + package_name + "/launch", [p]))

# 安装 config 文件（如果存在）
for p in glob("config/*.yaml"):
    if os.path.isfile(p):
        data_files.append(("share/" + package_name + "/config", [p]))

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    package_dir={"": "."},
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="root",
    maintainer_email="hongchao.shi@foxmail.com",
    description="Teleoperation package for TL series robotic arm via VR controller",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "tl_teleop = tl_teleop.tl_teleop:main",
        ],
    },
    cmdclass={"build_py": BuildPyWithSDK},
)
