#!/usr/bin/env python3

import configparser

fname = "../ci/platformio.ini"
action_dirs = "../../.github/workflows/"

template = """
name: Build examples for {derivate} @ {version}

on:
  push:
    branches: [ master ]
  pull_request:
    branches: [ master ]

jobs:
  build:

    runs-on: ubuntu-latest

    steps:
    - name: Checkout
      uses: actions/checkout@v3
    - name: Make directories
      run: bash extras/scripts/build-pio-dirs.sh
    - name: Build on PlatformIO
      run: bash extras/scripts/build-platformio.sh {derivate}{version}
"""

# Create a ConfigParser object
config = configparser.ConfigParser()

# Read the INI file
config.read(fname)

# Loop through sections
for section in config.sections():
    s = section.replace("env:","")
    flds = s.split("_")
    derivate = flds[0]
    version = s.replace(derivate + "_","")
    if version[0] == "V":
        print(f"Section: {derivate} {version}")
        action = template.format(derivate=derivate,version=version)
        fname = f"build_examples_{derivate}_{version}.yml"
        with open(action_dirs + fname,"w") as fp:
            fp.write(action)
