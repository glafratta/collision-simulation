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
l=np.empty(2)
linect=0
with open(file_list) as file:
    lines = file.read().splitlines()
    for line in lines:
        #print(line)
        linect+=1
        l=np.append(l,[linect, file])
        data= np.loadtxt(line, ndmin=2, delimiter="\t")
        if data.size !=0:
            bodies =np.append(bodies, data[:,0])
            branches =np.append(branches, data[:,1])
            if bodies[-1]>200 and branches[-1]>2:
                print(line)
            speed =np.append(speed, abs(data[:,2]))
    
#remove outliers
# print("hi")
speed_clean = np.empty(0)
branches_clean=np.empty(0)
bodies_clean=np.empty(0)
indices = np.ones(speed.size, dtype =bool)
for i in range(0,speed.size):
    if speed[i]>3.0 or branches[i]>100 or branches[i]<= 1 or bodies[i]<=1 or bodies[i]>200 or speed[i]<=0.05:
        #print(speed[i])
        indices[i]=0
        
# speed_clean = np.delete(speed, indices,0)
# branches_clean =np.delete(branches, indices,0)
# bodies_clean =np.delete(bodies,indices,0)
speed_clean = speed[indices]
branches_clean=branches[indices]
bodies_clean=bodies[indices]


#correlation, pearson's r


avg_bodies = np.sum(bodies_clean)/np.size(bodies_clean)
bodies_sd = np.std(bodies_clean)
avg_branches = np.sum(branches_clean)/np.size(branches_clean)
branches_sd = np.std(branches_clean)
avg_speed = np.sum(speed_clean)/np.size(speed_clean)
speed_sd = np.std(speed_clean)
bodies_correlation = np.corrcoef(bodies_clean, speed_clean)
branch_correlation = np.corrcoef(branches_clean, speed_clean)
branch_bodies_correlation = np.corrcoef(branches_clean, bodies_clean)

#inferential stats
speedplot= plt.figure()
axspeed = speedplot.add_subplot()
axspeed.set_ylabel("Speed (s)")
#axspeed.bar()


#other statistics
plt1= plt.figure()
plt2 = plt.figure()
plt3 =plt.figure()
ax1 = plt1.add_subplot()
#ax12=plt1.add_subplot()
ax1.set_xlabel("Bodies no.")
ax1.set_ylabel("Speed (s)")
ax2 = plt2.add_subplot()
ax3 = plt3.add_subplot()
ax1.scatter(bodies_clean, speed_clean)
BoSpcorrelationaxis =bodies_clean*bodies_correlation[1][0]
#ax1.axline((0,0), slope=bodies_correlation[1][0]*bodies_clean)
#ax1.plot(bodies_clean, bodies_clean*bodies_correlation[1][0])
ax1.plot(np. unique (bodies_clean), np. poly1d (np.polyfit (bodies_clean, speed_clean, 1))(np.unique (bodies_clean)), color = 'green')
ax2.scatter(branches_clean, speed_clean)
ax3.scatter(branches_clean, bodies_clean)
ax2.set_ylabel("Speed (s)")
ax2.set_xlabel("Branches no.")
ax3.set_ylabel("Bodies no.")
ax3.set_xlabel("Branches no.")
#plt.show()



print("speed-body correlation = " , bodies_correlation[1][0]) #circa .76 with branches >0
print("speed-ranch correlation = " , branch_correlation[1][0]) #circa .67 with branches >0
print("average speed: ", avg_speed, "SD = ", speed_sd)
print("average bodies: ", avg_bodies, "SD = ", bodies_sd)
print("avg branches: ", avg_branches, "SD: ", branches_sd)
print("min bodies:", min(bodies_clean), "max bodies: ", max(bodies_clean))
print("min speed:", min(speed_clean), "max bodies: ", max(speed_clean))
print("min branches:", min(branches_clean), "max branches: " ,max(branches_clean))
print("sample size:", len(bodies_clean), "og sample size: ", len(bodies))

f=open("results.txt", "w")
s=str(avg_speed)+" "+str(speed_sd)
f.write(s)
        
