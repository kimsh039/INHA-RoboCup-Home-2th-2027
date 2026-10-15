#!/usr/bin/env python3
"""Compatibility entry point for the timestamp-preserving C++ preprocessor."""
import os
from pathlib import Path
import sys

if __name__ == '__main__':
    launcher = str(Path(__file__).resolve().with_name('run-collision-cloud.sh'))
    os.execv('/bin/bash', ['bash', launcher, *sys.argv[1:]])
