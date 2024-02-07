#!/bin/bash

ls benchmark/stat* > data_list.txt

python3 speed_analysis.py
