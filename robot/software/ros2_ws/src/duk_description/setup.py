from setuptools import setup

setup(
    name="duk_description",
    version="0.1.0",
    packages=["duk_description"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/duk_description"]),
        ("share/duk_description", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    license="MIT",
    description="Geometry-derived static DUK preview",
    maintainer="DUK contributors",
    maintainer_email="maintainers@example.com",
)
