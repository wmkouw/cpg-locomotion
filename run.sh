#!/bin/bash
[ ! -d .venv ] && python3 -m venv .venv
source .venv/bin/activate
python3 $@
deactivate
