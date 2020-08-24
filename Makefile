# Makefile for building and installing the debian package for this repo
# Can also generate HTML docstrings from the *.rst files in docs/rst

PKGNAME = oresat-adcs-software
PKGVERSION = 0.0.1

.PHONY: help all helpers install docs uninstall cleandocs

# "make" alone is brought here
help:
	$(info This Makefile supports the following commands:)
	$(info all : Install the Debian package and generate HTML docstrings.)
	$(info cleandocs : Clean up any generated HTML docstrings.)
	$(info docs : Generate HTML pages containing this package's documentation.)
	$(info help : Display this message again.)
	$(info helpers : Downloads helper packages needed to install the oresat-adcs-software package.)
	$(info install : Compiles this source code into a Debian package and installs it on your machine. You should run "make helpers" before this command.)
	$(info uninstall : Uninstall the package and clear all docs.)	

all: helpers install docs

helpers:
	sudo apt install python3 python3-stdeb python-all dh-python

install:
	sudo python3 setup.py --command-packages=stdeb.command install_deb

# Assumes you made no changes to the package's structure or docstrings
# If you made changes that you would like to be shown in the HTML documentation,
# you need to run sphinx-apidoc to generate a new batch of *.rst files.
# See https://www.sphinx-doc.org/en/master/man/sphinx-apidoc.html for more info.
docs:
	mkdir docs/html
	sphinx-build -b html docs/rst docs/html

uninstall: cleandocs
	sudo rm -rf $(PKGNAME)-$(PKGVERSION).tar.gz $(PKGNAME)-$(PKGVERSION).egg-info deb_dist dist
	sudo apt-get remove python3-$(PKGNAME)

cleandocs:
	rm -rf docs/html
