#!/bin/bash
##temp=$(vcgencmd measure_temp | egrep -o '[0-9]*\.[0-9]*')
temp=0.0
 line=$(head -n 1 /sys/class/thermal/thermal_zone0/temp)
  # Test if the string is an integer as expected with a regular expression.
  if [[ $line =~ ^-?[0-9]+$ ]]
  then
    # Convert the CPU temperature to degrees Celsius and store as a string.
    temp=$(awk "BEGIN {printf \"%.2f\n\", $line/1000}")
fi
if [$temp <=80]
then
	echo "temperature ok, building"
	cmake .
	make
	sudo make install
else
	echo "too hot! temp = $temp , not building"
fi
