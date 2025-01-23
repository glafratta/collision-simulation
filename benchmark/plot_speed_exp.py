#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Feb  9 17:06:03 2024

@author: gula
"""
import matplotlib.pyplot as plt
import numpy as np
#import scipy as sp
speed_check=np.empty(1)

file= open("stats250624_1740.txt", "r")
lines = file.read().splitlines()
linect=0
lines_clen=[]

with open("stats250624_1740_check.txt", "w") as clean:
    for line in lines:
        skip=False
        for ch in line:
            if ch=='*' or  ch=='!':
                skip=True
            if skip==False:
                clean.write(line)
            
#    # l=np.append(l,[linect, file])

data= np.loadtxt("stats250624_1740_check.txt", ndmin=2, delimiter="\t")

if data.size !=0 & skip==0:
    # bodies =np.append(bodies, data[:,0])
    # branches =np.append(branches, data[:,1])
    # if bodies[-1]>200 and branches[-1]>2:
    #     print(line)
    speed_check =np.append(speed_check, abs(data[:,2]))



#speed= np.array([0.295, 0.166, .273, .133])
avg_check=np.sum(speed_check)/len(speed_check)
sd_check=np.std(speed_check)
speed= np.array([0.0335, 0.0160, .02269, .0048, 0.0057, 0.0048, 0.0056, avg_check])
SD=np.array([0, 0, 0, 0, 0, 0, 0, sd_check])
colors=["#c27ff5", "#68bcf7", "#add989", "#f5ed7f", "#892a58", "#0066cc", "#00ff80", "#cc66ff"]
foldername = ["t=0", "t=45", "t=116", "t=209", "t=302", "t=405", "t=500","check"]
#for directory in benchmark:
    #read results.txt
    #apend to speed and SD
    
speedplot= plt.figure(1)
ax = speedplot.add_subplot()
ax.set_ylabel("Speed (s)")
ax.bar(foldername, speed, color=colors)
ax.errorbar(foldername,speed, yerr=SD, fmt="o", color="b")

speedplot.savefig('back_forth_speed.eps', transparent=True, format='eps')

# speedplot2= plt.figure()
# ax2 = speedplot2.add_subplot()
# ax2.set_ylabel("Speed (s)")
# ax2.bar(foldername[2:], speed[2:], color=colors[2:])
# ax2.errorbar(foldername[2:],speed[2:], yerr=SD[2:], fmt="o", color="b")

# speedplot2.savefig('target_speed.eps', transparent=True, format='eps')
