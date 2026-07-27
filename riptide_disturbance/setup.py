from setuptools import find_packages, setup

package_name = "riptide_disturbance"

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
    maintainer="Nepomuk Werner",
    maintainer_email="nepomukwerner23@gmail.com",
    description="Disturbance scenario generator for the Riptide sim.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "disturbance_generator = riptide_disturbance.disturbance_generator:main",
        ],
    },
)
