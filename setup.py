""" OreSat ADCS Manager Setup.py """

from setuptools import setup
import adcs_manager

with open("README.md", "r") as f:
    long_description = f.read()

setup(
    name=adcs_manager.APP_NAME,
    version=adcs_manager.APP_VERSION,
    author=adcs_manager.APP_AUTHOR,
    license=adcs_manager.APP_LICENSE,
    description=adcs_manager.APP_DESCRIPTION,
    long_description=long_description,
    author_email="rmedick@pdx.edu",
    maintainer="Ryan Medick",
    maintainer_email="rmedick@pdx.edu",
    url="https://github.com/oresat/oresat-adcs-manager",
    packages=['adcs_manager'],
    install_requires=[
        "pydbus",
    ],
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: GNU General Public License v3 (GPLv3)",
    ],
    entry_points={
        "console_scripts": [
            "oresat-adcs-managerd = adcs_manager.__main__"
        ]
    },
)
