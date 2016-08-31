#!/bin/bash
if [ $_ == $0 ]
then
    echo "Please source this script. Do not execute."
    exit 1
fi
. ../.env/bin/activate
