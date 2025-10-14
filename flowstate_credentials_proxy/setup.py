from setuptools import find_packages, setup

package_name = "flowstate_credentials_proxy"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test_*.py"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "aiohttp"],
    zip_safe=True,
    maintainer="koonpeng",
    maintainer_email="koonpeng@intrinsic.ai",
    description="websocket proxy to authenticate to flowstate",
    license="Intrinsic License",
    entry_points={
        "console_scripts": [
            "credentials_proxy = flowstate_credentials_proxy.__main__:main",
        ],
    },
)
