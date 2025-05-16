from setuptools import find_packages, setup

package_name = "helmoro_cli"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Marc Blöchlinger",
    maintainer_email="mbloechli@student.ethz.ch",
    description="Command line interface for Helmoro",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "open_cli = helmoro_cli.main:main",
        ],
    },
)
