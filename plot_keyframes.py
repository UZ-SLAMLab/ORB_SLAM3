import csv
from mpl_toolkits import mplot3d
import numpy as np
import matplotlib
# matplotlib.use('TkAgg')
import matplotlib.pyplot as p

# python3 -m pip install numpy matplotlib 
# brew install python-tk in case you cannot plot because using agg

# filename = 'KeyFrameTrajectory.txt' 
file = 'sampleDataFramesKeyframe'
filename = file + '.txt'
savefile = 'figs/' + file + '.png'
x, y, z = [], [], []
with open(filename, newline='') as csvfile:
    r = csv.reader(csvfile, delimiter=' ', quotechar='|')
    xyzs = [row[1:4] for row in r]
    for x_, y_, z_ in xyzs:
        x.append(float(x_))
        y.append(float(y_))
        z.append(float(z_))


fig = p.figure()
ax = p.axes(projection='3d')
ax.scatter3D(x, y, z, 'purple')
# ax.plot3D(x, y, z, 'red')

ax.plot3D(x[1],y[1],z[1], '-o', markersize=30,markerfacecolor='red')
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
p.savefig(savefile)
p.show()

