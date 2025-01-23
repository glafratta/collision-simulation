#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Sep 23 14:06:46 2024

@author: gula
"""
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Feb  9 17:06:03 2024

@author: gula
"""
import matplotlib.pyplot as plt
import numpy as np
from scipy import stats

speed_explore= np.array([0.068485, 0.086361, 0.0603, 0.059177,0.066586])
speed_recall=np.array([0.002843, 0.002545, 0.00239,0.002453,0.002499])
mu_ex=np.sum(speed_explore)/len(speed_explore)
SD_ex=np.std(speed_explore)
mu_recall=np.sum(speed_recall)/len(speed_recall)
SD_recall=np.std(speed_recall)
mu=[mu_ex, mu_recall]
SD=[SD_ex, SD_recall]
colors=["#c27ff5", "#68bcf7"] #, "#0066cc", "#00ff80", "#cc66ff", "#add989", "#f5ed7f", "#892a58"
foldername = ["exploration", "recall" ] #"t=116", "t=209", "t=302", "t=405", "t=500","check"
#for directory in benchmark:
    #read results.txt
    #apend to speed and SD
    
speedplot= plt.figure(1)
ax = speedplot.add_subplot()
ax.set_ylabel("Time (s)")
ax.bar(foldername, mu, color=colors)
ax.errorbar(foldername,mu, yerr=SD, fmt="o", color="b")

T, p=stats.ttest_ind(speed_explore, speed_recall)

#speedplot.savefig('explore_vs_graph.eps', transparent=True, format='eps')

# speedplot2= plt.figure()
# ax2 = speedplot2.add_subplot()
# ax2.set_ylabel("Speed (s)")
# ax2.bar(foldername[2:], speed[2:], color=colors[2:])
# ax2.errorbar(foldername[2:],speed[2:], yerr=SD[2:], fmt="o", color="b")

# speedplot2.savefig('target_speed.eps', transparent=True, format='eps')

