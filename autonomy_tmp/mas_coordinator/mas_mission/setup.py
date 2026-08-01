from setuptools import setup

package_name = "mas_mission"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/team_launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            f"mission_logic_node = {package_name}.mission_logic_node:main",
            f"stub_explorer = {package_name}.stub_explorer:main",
        ],
    },
)
