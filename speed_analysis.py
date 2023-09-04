#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Aug 20 16:06:54 2023

@author: gula
"""
import numpy as np
import matplotlib.pyplot as plt
import scipy as sp

file_list = "data_list.txt"
bodies =np.empty(1)
branches=np.empty(1)
speed=np.empty(1)
#data = np.empty(3)
#_data = np.empty(3)
with open(file_list) as file:
    lines = file.read().splitlines()
    for line in lines:
        #print(line)
        data= np.loadtxt(line, ndmin=2, delimiter="\t")
        if data.size !=0:
            bodies =np.append(bodies, data[:,0])
            branches =np.append(branches, data[:,1])
            speed =np.append(speed, abs(data[:,2]))
    
#remove outliers
# print("hi")
speed_clean = np.empty(1)
branches_clean=np.empty(1)
bodies_clean=np.empty(1)
indices = np.ones(speed.size, dtype =bool)
for i in range(0,speed.size):
    if speed[i]>3.0:
        #print(speed[i])
        indices[i]=0
# speed_clean = np.delete(speed, indices,0)
# branches_clean =np.delete(branches, indices,0)
# bodies_clean =np.delete(bodies,indices,0)
speed_clean = speed[indices]
branches_clean=branches[indices]
bodies_clean=bodies[indices]


avg_bodies = np.sum(bodies_clean)/np.size(bodies_clean)
bodies_sd = np.std(bodies_clean)
avg_branches = np.sum(branches_clean)/np.size(branches_clean)
avg_speed = np.sum(speed_clean)/np.size(speed_clean)
speed_sd = np.std(speed_clean)
plt1= plt.figure()
plt2 = plt.figure()
plt3 =plt.figure()
ax1 = plt1.add_subplot()
ax1.set_xlabel("Bodies")
ax1.set_ylabel("Speed")
ax2 = plt2.add_subplot()
ax3 = plt3.add_subplot()
ax1.scatter(bodies_clean, speed_clean)
ax2.scatter(branches_clean, speed_clean)
ax3.scatter(branches_clean, bodies_clean)
ax2.set_ylabel("Speed")
ax2.set_xlabel("Branches")
ax3.set_ylabel("Bodies")
ax3.set_xlabel("Branches")
#plt.show()

#correlation, pearson's r
bodies_correlation = np.corrcoef(bodies_clean, speed_clean)
branch_correlation = np.corrcoef(branches_clean, speed_clean)
branch_bodies_correlation = np.corrcoef(branches_clean, bodies_clean)
print("speed-body correlation = " , bodies_correlation[1][0]) #circa .76 with branches >0
print("speed-ranch correlation = " , branch_correlation[1][0]) #circa .67 with branches >0
print("average speed: ", avg_speed, "SD = ", speed_sd)
print("average bodies: ", avg_bodies, "SD = ", bodies_sd)



        
