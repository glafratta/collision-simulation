#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Oct  9 15:01:08 2024

@author: gula
"""
import numpy as np
import sys

file_txt = "data_list.txt"
file_name = sys.argv[1]
bodies =np.array([], ndmin=1)
states=np.array([], ndmin=1)
time=np.array([],ndmin=1)
#data = np.empty(3)
#_data = np.empty(3)
l=np.empty(2)
filect=0
new_file= file_name+ ".txt"
open(new_file, "w")
with open(file_txt) as file_list:
    files = file_list.read().splitlines()
    for file in files:
        filect+=1
        l=np.append(l,[filect, file_list])
        #clean up
        file_n=file
        file= open(file, "r")
        lines = file.read().splitlines()
        with open(new_file, "a") as clean:
            if len(lines)==0:
                print(file_n, "empty")
            for line in lines:
                if line[0]=='*':
                    line=line[1:]
                    clean.write(line)
                    clean.write('\n')

with open(new_file, "r") as clean_r:
        data= np.loadtxt(clean_r, ndmin=2, delimiter="\t")
        if data.size !=0:
            bodies =np.append(bodies, data[-11:,0])
            states =np.append(states, data[-11:,1])
            if bodies[-1]>200 and branches[-11]>2:
                print(line)
            time =np.append(time, abs(data[-11:,2]))


avg_bodies = np.sum(bodies)/np.size(bodies)
bodies_sd = np.std(bodies)
avg_states = np.sum(states)/np.size(states)
states_sd = np.std(states)
avg_time = np.sum(time)/np.size(time)
time_sd = np.std(time)

print(avg_bodies ,"+/-", bodies_sd, " bodies")
print("min bodies= ", min(bodies), " max bodies= ", max(bodies))
print(avg_states ,"+/-", states_sd, " states")
print("min states= ", min(states), " max states= ", max(states))
print(avg_time ,"+/-", time_sd, " s")
print("min time= ", min(time), " max time= ", max(time))
print("N= ", len(time))
print(time)





