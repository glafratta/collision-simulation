import numpy as np
import matplotlib.pyplot as plt
import scipy as sp

framerate=30
nyquist=framerate/2
#with open("noise.txt") as file:
   # lines = file.read().splitlines()
    #for line in lines: 
data= np.loadtxt("noise.txt", ndmin=2, delimiter="\t")
data_avg=np.loadtxt("noise_avg.txt", ndmin=2, delimiter="\t")
data_iir_still=np.loadtxt("still_iir.txt", ndmin=2, delimiter="\t")
data_iir_straight=np.loadtxt("straight_iir.txt", ndmin=2, delimiter="\t")
data_s= np.loadtxt("avg_s_1m.txt", ndmin=2, delimiter="\t")

x= data[:,0]
y=data[:,1]

x_avg= data_avg[:,0]
y_avg=data_avg[:,1]

x_iir=data_iir_straight[2:320,0]
y_iir=data_iir_straight[:320,1]
signal_unfiltered=data_iir_straight[:320,4]

x_s= data_s[:264,0]
y_s=data_s[:264,1]

x_fft= np.fft.fft(x)/len(x)
x_axisx=np.linspace(0, framerate, len(x))

x_avg_fft= np.fft.fft(x_avg)/len(x_avg)
x_axisx_avg=np.linspace(0, framerate, len(x_avg))



x_s_fft= np.fft.fft(x_s)/len(x_s)
x_s_axisx=np.linspace(0, framerate, len(x_s))

n=len(x_s_fft)
denoised_s=x_s_fft.copy()
denoised=x_fft.copy()
indices= [[0,1], [3,nyquist]]
for step in range(0, len(indices)):
    k_low1 = int(len(x_s_fft)/framerate*indices[step][0])
    k_hi1 = int(len(x_s_fft)/framerate*indices[step][-1])
    denoised_s[n-k_hi1:n-k_low1+1] =0
    denoised[n-k_hi1:n-k_low1+1] =0
    denoised_s[k_low1:k_hi1+1]=0
    denoised[k_low1:k_hi1+1]=0

x_denoised = np.real(np.fft.ifft(denoised))*len(denoised)
x_s_denoised = np.real(np.fft.ifft(denoised_s))*len(denoised_s)

