from glob import glob
from setuptools import find_packages, setup


package_name = "tl_hand_deploy"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="nvidia",
    maintainer_email="nvidia@localhost",
    description="TCB605 and ROH-LiteS001 guarded deployment utilities",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "tl_safe_arm = tl_hand_deploy.safe_arm_node:main",
            "tl_safe_move = tl_hand_deploy.safe_move_cli:main",
            "tl_hand_guard = tl_hand_deploy.roh_hand_node:main",
            "tl_handeye_collect = tl_hand_deploy.handeye_collect_node:main",
            "tl_handeye_calibrate = tl_hand_deploy.handeye_calibrate:main",
            "tl_handeye_accuracy = tl_hand_deploy.handeye_accuracy:main",
            "tl_handeye_validate = tl_hand_deploy.handeye_validate_report:main",
            "tl_overhead_test = tl_hand_deploy.overhead_test:main",
            "tl_overhead_movel_test = tl_hand_deploy.overhead_movel_test:main",
            "tl_mock_place_teach = tl_hand_deploy.mock_place_teach:main",
            "tl_mock_pick_place = tl_hand_deploy.mock_pick_place:main",
            "tl_mock_pick_place_movej = tl_hand_deploy.mock_pick_place_movej:main",
            "tl_aruco_basket_teach = tl_hand_deploy.aruco_basket_teach:main",
            "tl_system_check = tl_hand_deploy.system_check:main",
            "tl_convert_handeye_frame = tl_hand_deploy.convert_handeye_frame:main",
        ]
    },
)
