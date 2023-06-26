#!/bin/bash
temp = $(vcgencmd measure_temp | egrep -o '[0-9]*\.[0-9]*')
if [$temp<80]
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
