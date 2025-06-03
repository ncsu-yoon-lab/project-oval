import json 
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

with open('jsonscan/scan_1739048517_374113717.json', 'r') as f:
    pcd = json.load(f)

fig = plt.figure(figsize=(10,7))
ax = fig.add_subplot(111, projection='3d')

x_list = []
y_list = []
z_list = []

for i in range(len(pcd['data'])):
    x_list.append(pcd['data'][i][0])
    y_list.append(pcd['data'][i][1])
    z_list.append(pcd['data'][i][2])

ax.scatter(x_list, y_list, z_list, c='blue', s=1, alpha=0.5)

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

plt.show()