# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.

import os
import sys
sys.path.insert(0, os.path.abspath('/home/jh42/oresat/'))
sys.path.insert(0, os.path.abspath('/home/jh42/oresat/oresat_adcs_software'))
sys.path.insert(0, os.path.abspath('/home/jh42/oresat/oresat_adcs_software/adcs_manager'))
sys.path.insert(0, os.path.abspath('/home/jh42/oresat/oresat_adcs_software/adcs_manager/adcs_lib'))
sys.path.insert(0, os.path.abspath('/home/jh42/oresat/oresat_adcs_software/adcs_42_interface'))

# -- Project information -----------------------------------------------------

project = 'ADCS Software'
copyright = '2020, Portland State Aerospace Society'
author = 'PSAS'


# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = ["sphinx.ext.autodoc"]

# Add any paths that contain templates here, relative to this directory.
templates_path = []

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = []


# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
html_theme = 'classic'

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = []

# Example configuration for intersphinx: refer to the Python standard library.
intersphinx_mapping = {'python': ('https://docs.python.org/3/', None)}
