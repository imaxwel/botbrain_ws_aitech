from glob import glob
import os

from setuptools import find_packages, setup


package_name = "botbrain_ws_gateway"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools", "fastapi", "uvicorn", "pyyaml"],
    zip_safe=True,
    maintainer="tour-guide-robot",
    maintainer_email="robotics@example.com",
    description="HTTP bridge sidecar for Mission Supervisor to control the G1 botbrain_ws_aitech runtime.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "botbrain_ws_gateway = botbrain_ws_gateway.main:main",
        ],
    },
)
