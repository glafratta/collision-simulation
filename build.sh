#!/bin/bash
temp=$(vcgencmd measure_temp | egrep -o '[0-9]*\.[0-9]*')
if [$temp <=80]
then
	echo "temperature ok, building"
	cmake .
	make
	sudo make install
else
	echo "too hot! temp = $temp , not building"
fi
