#!/bin/bash

ls bodiesSpeedStats*/stat* > data_list.txt

python3 speed_analysis.py
