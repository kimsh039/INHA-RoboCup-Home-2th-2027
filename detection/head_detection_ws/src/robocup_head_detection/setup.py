from glob import glob
from setuptools import find_packages, setup

name = "robocup_head_detection"
setup(
    name=name, version="0.1.0", packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + name]),
        ("share/" + name, ["package.xml"]),
        ("share/" + name + "/config", glob("config/*.yaml")),
        ("share/" + name + "/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"], zip_safe=True,
    maintainer="INHA RoboCup", maintainer_email="kimsh039@users.noreply.github.com",
    description="Head detection and verified tracking", license="Apache-2.0",
    entry_points={"console_scripts": [
        "head_detection_node = robocup_head_detection.node:main",
        "detection_nav_goal_node = robocup_head_detection.nav_goal_node:main",
        "support_surface_node = robocup_head_detection.support_surface_node:main",
    ]},
)
