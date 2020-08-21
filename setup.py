import setuptools

if __name__ == "__main__":
    with open("README.md", "r") as fh:
        long_description = fh.read()

    setuptools.setup(
        name="oresat-adcs-software",
        version="0.0.1",
        author="",
        author_email="",
        description="A daemon for managing the ADCS on OreSat.",
        long_description=long_description,
        long_description_content_type="text/markdown",
        url="https://github.com/oresat/oresat-adcs-software",
        packages=setuptools.find_packages(),
        classifiers=[
            "Programming Language :: Python :: 3",
            "License :: OSI Approved :: GPLv3 License",
        ],
        python_requires='>=3.7',
        entry_points={
            "console_scripts": [
                "manager = adcs_manager.main",
                "42 = adcs_42_interface.main"
            ]
        }
    )
