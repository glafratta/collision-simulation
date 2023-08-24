#!/bin/bash
##temp=$(vcgencmd measure_temp | egrep -o '[0-9]*\.[0-9]*')
temp=$(head -n 1 /sys/class/thermal/thermal_zone0/temp)
  # Test if the string is an integer as expected with a regular expression.
#  if [ $line =~ ^-?[0-9]+$ ]
#  then
    # Convert the CPU temperature to degrees Celsius and store as a string.
#    temp=$(awk "BEGIN {printf \"%.2f\n\", $line/1000}")
#fi
if [ $temp -lt 51000 ]
then
	echo "temperature of $temp ok, building"
	rm targetless targetless_benchmark navigate targetless_simple
	cmake .
	cd src/
	sudo make install
	cd ..
	make
else
	echo "too hot! temp = $temp , not building"
fi
