#!/usr/bin/env python

import re
import os
import pathlib
from distutils.core import setup

# Determine the list of packages
packages = [("linelidarclass." + \
			".".join(pathlib.Path(e[0]).parts)).rstrip(".") \
		for e in os.walk(".") if "__init__.py" in e[2]]

# Determine the version of the class from the entries in the Changelog
version_major = 0
version_minor = 0
version_micro = 0

with open("Changelog.txt", "r") as src:
  for l in src:
    m = re.findall("^\s*Version\s+([0-9]+)\.([0-9]+)\.([0-9]+)", l)
    if m:
      major = int(m[-1][0])
      minor = int(m[-1][1])
      micro = int(m[-1][2])
      if major > version_major or (major == version_major and \
		(minor > version_minor or (minor == version_minor and \
		micro > version_micro))):
        version_major = major
        version_minor = minor
        version_micro = micro

version = "{}.{}.{}".format(version_major, version_minor, version_micro)

# Setup
setup(name = "linelidarclass",
	version = version,
	description = "LineLidar communication class",
	author = "Noptel Oy",
	author_email = "pierre.coupard@noptel.fi",
	package_dir = {"linelidarclass": ""},
	packages = packages)
