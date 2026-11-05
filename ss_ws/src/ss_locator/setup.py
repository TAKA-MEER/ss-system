from setuptools import setup

package_name = "ss_locator"

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
    maintainer="ss-system maintainers",
    maintainer_email="maintainer@example.com",
    description="Panel locator node and estimation package.",
    license="AGPL-3.0-only",
    tests_require=["pytest"],
    entry_points={"console_scripts": []},
)
