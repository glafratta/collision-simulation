#!/bin/bash
temp=$(head -n 1 /sys/class/thermal/thermal_zone0/temp)
if [ $temp -lt 70000 ]
echo "temperature = $temp"
then
	echo "ok to run, running"
	echo "planning on? answer 0 or 1"
	read planning
	echo "debug on? answer 0 or 1"
	read debug
	if [debug==0]
	then
		sudo ./navigate $debug $planning
	else
		echo "what file to write outputs to?"
		read filename
		sudo ./navigate $debug $planning $filename 
else
	echo "too hot!"
fi
