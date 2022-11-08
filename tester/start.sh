#!/usr/bin/bash
git pull
git submodule update
./gui.py >>log.txt
