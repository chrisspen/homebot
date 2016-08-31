#!/bin/bash

for session in $(screen -ls | grep -oP '(?<=\s)([0-9]+)(?=\.\.)')
do
    screen -S "${session}" -X quit;
done

kill `ps aux|grep -i "/ros/"|grep -v grep|awk '{print $2}'|xargs`

sleep 5
