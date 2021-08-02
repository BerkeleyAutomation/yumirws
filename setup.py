"""
Setup of YuMi RWS python codebase
Author: Mike Danielczuk
"""
import os
from setuptools import setup, find_packages

requirements = [
    "numpy",
    "autolab-core",
#     "abb_librws", TODO: add pip package for this lib
]

# load __version__ without importing anything
version_file = os.path.join(
    os.path.dirname(__file__), "yumirws/version.py"
)
with open(version_file, "r") as f:
    # use eval to get a clean string of version from file
    __version__ = eval(f.read().strip().split("=")[-1])

# load README.md as long_description
long_description = ""
if os.path.exists("README.md"):
    with open("README.md", "r") as f:
        long_description = f.read()

setup(
    name="yumirws",
    version=__version__,
    description="YuMi Robot Web Services (RWS) wrapper for abb_librws library",
    long_description=long_description,
    author="Mike Danielczuk/Justin Kerr",
    author_email="mdanielczuk@berkeley.edu, jkerr@berkeley.edu",
    license="MIT Software License",
    url="https://github.com/BerkeleyAutomation/yumirws",
    keywords="robotics grasping",
    classifiers=[
        "Development Status :: 4 - Beta",
        "License :: OSI Approved :: MIT Software License",
        "Programming Language :: Python",
        "Programming Language :: Python :: 3.6",
        "Programming Language :: Python :: 3.7",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Natural Language :: English",
        "Topic :: Scientific/Engineering",
    ],
    packages=find_packages(),
    install_requires=requirements,
)