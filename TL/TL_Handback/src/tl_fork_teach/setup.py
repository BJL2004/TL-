from setuptools import find_packages, setup


package_name = "tl_fork_teach"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="nvidia",
    maintainer_email="nvidia@localhost",
    description="Standalone U-fork basket teaching tool for TL TCB605",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "tl_fork_basket_teach = tl_fork_teach.fork_basket_teach:main",
            "tl_fork_basket_cycle = tl_fork_teach.fork_basket_cycle:main",
            "tl_fork_trim_mqtt = tl_fork_teach.fork_trim_mqtt:main",
        ]
    },
)
