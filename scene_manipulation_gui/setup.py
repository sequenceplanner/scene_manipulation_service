from setuptools import setup

package_name = "scene_manipulation_gui"

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
    author="Martin Dahl, Kristofer Bengtsson, Endre Erős",
    author_email="martin.dahl@chalmers.se, kristofer.bengtsson@chalmers.se, endree@chalmers.se",
    maintainer="Martin Dahl, Kristofer Bengtsson, Endre Erős",
    maintainer_email="martin.dahl@chalmers.se, kristofer.bengtsson@chalmers.se, endree@chalmers.se",
    keywords=["ROS2", "Volvo Rita"],
    classifiers=[
        "Intended Audience :: Developers",
        "Programming Language :: Python",
        "Topic :: Software Development",
    ],
    description="Package containing different Graphical User Interfaces for the Volvo Rita project.",
    license="",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "scene_manipulation_gui = scene_manipulation_gui.scene_manipulation_gui:main"
        ],
    },
)
