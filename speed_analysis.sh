#!/bin/bash

for file in benchmark/testdelete/; do
if [$(wc -l <file)-]; then
	rm file
fi
done


ls benchmark/stat* > data_list.txt

python3 speed_analysis.py
