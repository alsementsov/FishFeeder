import numpy as np
import matplotlib.pyplot as plt
from numpy import genfromtxt
import scipy.signal as sig
from tkinter import Tk
from tkinter.filedialog import askopenfilename
import pylab
Q = 0.1#0.00001  #SKF process variance   -5 
R = 0.01  # SKF estimate of measurement variance, change to see effect

def KalmanFilter(mas,Qf,Rf,sz):
    # allocate space for arrays
    x=np.zeros(sz)      # a posteri estimate of x
    P=np.zeros(sz)         # a posteri error estimate
    xhatminus=np.zeros(sz) # a priori estimate of x
    Pminus=np.zeros(sz)    # a priori error estimate
    K=np.zeros(sz)         # gain or blending factor
    # intial guesses
    x[0] = mas[0]
    P[0] = 1.0
    
    for k in range(1,sz):
        # time update
        xhatminus[k] = x[k-1]
        Pminus[k] = P[k-1]+Qf
        # measurement update
        K[k] = Pminus[k]/( Pminus[k]+Rf)
        x[k] = xhatminus[k]+K[k]*(mas[k]-xhatminus[k])
        P[k] = (1-K[k])*Pminus[k]
    return x
mas = np.array([1000,400,300,180,3000,6000,2000,1500,800,1000,1200,2000,3000,4000,2000,800,300,1000,900,1300,1500,900,1100,1000,900,800,1200])
mas_shape = np.shape(mas);
N=mas_shape[0]
print(N)

kf10 = KalmanFilter(mas,Q/10,R,N)
kf100 = KalmanFilter(mas,Q/100,R,N)
kf = KalmanFilter(mas,Q/N,R,N)

 
 
plt.grid(axis='both',linestyle = '--')
plt.plot(mas,color='black',linewidth ='3')#errors
plt.plot(kf10,color='blue',linewidth ='2')#errors
plt.plot(kf100,color='red',linewidth ='2')#errors
plt.plot(kf,color='green',linewidth ='2')#errors

figManager = plt.get_current_fig_manager()
figManager.window.showMaximized()
plt.show()