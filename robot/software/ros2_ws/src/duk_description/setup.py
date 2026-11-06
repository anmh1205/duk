from setuptools import setup

setup(
    name="duk_description",
    version="0.1.0",
    packages=["duk_description"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/duk_description"]),
        ("share/duk_description", ["package.xml"]),
        ("share/duk_description/launch", ["launch/kinematics.launch.py"]),
        ("share/duk_description/config", ["config/kinematics.rviz"]),
    ],
    entry_points={
        "console_scripts": ["kinematic_bench = duk_description.kinematic_node:main"]
    },
    install_requires=["setuptools"],
    license="MIT",
    description="Geometry-derived static DUK preview",
    maintainer="DUK contributors",
    maintainer_email="maintainers@example.com",
)
