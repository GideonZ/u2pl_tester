#!/usr/bin/bash
git pull
git submodule update
make -C ../ecpprog/ecpprog
./gui.py >>/dev/null
