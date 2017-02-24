import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from random import random,seed

seed(1)

def randomColor():
	letters = '0123456789abcdef'
	return "#"+reduce(lambda x,y:x+y,[letters[int(random()*6)] for i in range(6)],'')

def column(matrix, i):
    return [row[i] for row in matrix]

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
colorarr = "cmykrgbp"

#reading no. of clusters from file 
with open ("number_of_scatter_points.txt") as f:
    lim=f.readlines()
limit=int (lim) 
for i in range(0,limit):
    with open('Cluster'+str(i)+'.txt') as file:
        array2d = [[float(digit) for digit in line.split()] for line in file]
        x = column(array2d,0)
        y = column(array2d,1)
        z = column(array2d,2)
        ax.scatter(x,y,z,c=randomColor(),marker='+')

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_autoscale_on(False)
plt.show()

