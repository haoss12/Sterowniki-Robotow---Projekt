import numpy as np
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt


rawData = np.genfromtxt('mag2SRP.txt',
                        delimiter=' ')
rawDataC = np.genfromtxt('Cmag2SRP.txt',
                        delimiter=' ')

N = len(rawData)
print(N)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_box_aspect([1,1,1])
ax.view_init(elev=20, azim=80)
for i in range(N):
    xraw = rawData[i, 0]
    yraw = rawData[i, 1]
    zraw = rawData[i, 2]

    ax.scatter(xraw, yraw, zraw, color='r')

N = len(rawDataC)
print(N)
for i in range(N):
    xraw = rawDataC[i, 0]
    yraw = rawDataC[i, 1]
    zraw = rawDataC[i, 2]

    ax.scatter(xraw, yraw, zraw, color='b')

plt.show()