#!/bin/bash
temp=$(head -n 1 /sys/class/thermal/thermal_zone0/temp)
if [ $temp -lt 51000 ]
then
	echo "ok to run, running"
	echo "planning on? answer 0 or 1"
	read planning
	echo "debug on? answer 0 or 1"
	read debug
	sudo ./navigate $planning $debug
else
	echo "too hot! temperature = $temp"
fi
