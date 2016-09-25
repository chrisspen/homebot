#!/bin/bash
ps aux|grep -i "/ros/"|grep -v grep
RET=$?
if [ $RET -eq 0 ]
then
    # Found something.
    echo 'Homebot is running.'
else
    # Nothing running.
    echo 'Homebot is stopped.'
fi
