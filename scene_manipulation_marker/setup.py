from setuptools import setup

package_name = "scene_manipulation_marker"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    author="Endre Erős",
    author_email="endree@chalmers.se",
    maintainer="Endre Erős",
    maintainer_email="endree@chalmers.se",
    keywords=["ROS2"],
    classifiers=[
        "Intended Audience :: Developers",
        "Programming Language :: Python",
        "Topic :: Software Development",
    ],
    description="A marker to save poses.",
    license="",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "scene_manipulation_marker = scene_manipulation_marker.scene_manipulation_marker:main",
        ],
    },
)
