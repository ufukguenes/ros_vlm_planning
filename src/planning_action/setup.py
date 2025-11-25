from setuptools import find_packages, setup

package_name = "planning_action"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ufuk",
    maintainer_email="u.guenes@live.de",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "server = planning_action.server:main",
            "dummy_img_publisher = planning_action.dummy_img_publisher:main"
        ],
    },
)
