#!/bin/bash

##iterate through folders
COUNT=0
for folder in 210522_*; do
	cd $folder;
	pwd;
	for file in map*;do
		../turnMaps $file;
	done;
	cd ..;
	echo "back to "; 
	pwd;
	
done
##iterate through files in folders
##for each file in folder run ../../test/
