#!/bin/bash
cpplint --recursive --exclude=*build --extensions=c,cc,h,hpp,c++,h++,cu,cpp,hxx,cxx,cuh,ino \
    $( find src/ros/src/ros_homebot -name \*.cpp -or -name \*.ino | grep -vE \.build | grep -vE Adafruit | grep -vE examples | grep -vE lib )
