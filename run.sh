#!/bin/bash
temp=$(head -n 1 /sys/class/thermal/thermal_zone0/temp)
echo "temperature = $temp"
if [ $temp -lt 60000 ]
then
	echo "ok to run, running"
	echo "planning on? answer 0 or 1"
	read planning
	echo "debug on? answer 0 or 1"
	read debug
	if [ debug ==0 ]
	then
	sudo ./navigate $debug $planning
	else
	echo "what file to write to?"
	read filename
	sudo ./navidate $debug $planning > /tmp/$filename
	fi
else
	echo "too hot! wait"
fi
