#!/bin/bash
# CS 2017.4.14
# Only run pep8 checks on locally modified files.
# Note, this won't find files in directories that haven't been added to git's index,
# so be sure to run `git status` to review changes and `git add .` to add if appropriate.
FILES=`git status --porcelain | grep -E "*\.py$" | grep -v migration | grep -v " D " | awk '{print $2}'`
echo "Checking: $FILES"
VENV=${VENV:-.env}
$VENV/bin/pylint --rcfile=pylint.rc $FILES
