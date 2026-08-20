from setuptools import find_packages, setup

package_name = "guide_mission"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/floors.example.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jongky",
    maintainer_email="you@example.com",
    description="안내로봇 임무 상태머신 (층 전환)",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "check_floors = guide_mission.check_floors:main",
        ],
    },
)
