from glob import glob

from setuptools import setup

package_name = "victim_detector_integrated"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/worlds", glob("worlds/*")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        # 权重必须装进 share，否则 launch 里的默认权重路径指向不存在的文件
        ("share/" + package_name + "/models", glob("models/*.pt")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Jian",
    maintainer_email="a1955496@adelaide.edu.au",
    description="Minimal ROS 2 victim detection node for the IEEE SMCS SAR competition",
    license="MIT",
    entry_points={
        "console_scripts": [
            "detector_node = victim_detector_integrated.detector_node:main",
        ],
    },
)
