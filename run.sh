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
	if [ $debug -eq 0 ]
	then
	sudo ./navigate $debug $planning
	else
	today =$(date +'%d%m%Y_%H%M')
	filename = $("/tmp/transcript$date"+".txt")
	echo "writing to $filename"
	sudo ./navigate $debug $planning > $filename
	fi
else
	echo "too hot! wait"
fi
