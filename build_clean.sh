#!/bin/bash

# Clean build script for LineExtraction project
# Cleans build directory and rebuilds with 8 parallel jobs

cd /workspace/LineExtraction/build && make clean && cmake --build . -j 8
