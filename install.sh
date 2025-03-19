#!/bin/bash -e
[ ! -d .venv ] && python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install --upgrade pip
python3 -m pip install numpy matplotlib pybullet
deactivate
