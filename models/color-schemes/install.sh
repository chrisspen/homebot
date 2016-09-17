#!/bin/bash
# 2016.9.13 CKS
# Installs custom color schemes into your user-configurable OpenSCAD settings.
# For more info see:
#
#   https://github.com/openscad/openscad/wiki/Path-locations
#
mkdir -p ~/.config/OpenSCAD/color-schemes/render
ln -s $(pwd)/render/white.json ~/.config/OpenSCAD/color-schemes/render
