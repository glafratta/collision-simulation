#!/bin/bash

for file in benchmark/testdelete/; do
if [$(wc -l <file)-]; then
	rm file
fi
done


ls Dmatch_target_check*/stat* > data_list.txt

python3 planning_only.py
