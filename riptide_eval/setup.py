from setuptools import setup

package_name = "riptide_eval"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Nepomuk Werner",
    maintainer_email="nepomukwerner23@gmail.com",
    description="Benchmark runner for the Riptide control zoo (CSV + plots).",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "benchmark = riptide_eval.benchmark:main",
        ],
    },
)
