import csv
from mpl_toolkits import mplot3d
import numpy as np
import matplotlib
# matplotlib.use('TkAgg')
import matplotlib.pyplot as p
import open3d as o3d
# python3 -m pip install numpy matplotlib open3d
# brew install python-tk in case you cannot plot because using agg

# filename = 'KeyFrameTrajectory.txt' 
file = 'my_pcd3'
filename = file + '.txt'
pcdname = file + '.pcd'
savefile = 'figs/' + file + '.png'
x, y, z = [], [], []
points = []

print("Load a ply point cloud, print it, and render it")
pcd = o3d.io.read_point_cloud(pcdname)
print(pcd)
print(np.asarray(pcd.points))
o3d.visualization.draw_geometries([pcd])
# with open(filename, newline='') as csvfile:
#     r = csv.reader(csvfile, delimiter=' ', quotechar='|')
#     xyzs = [row[1:4] for row in r]
#     for x_, y_, z_ in xyzs:
#         x.append(float(x_))
#         y.append(float(y_))
#         z.append(float(z_))
        
#         # points = [[points],[x_,y_,z_]]


# fig = p.figure()
# ax = p.axes(projection='3d')
# # ax.scatter3D(x, y, z, 'purple')
# ax.plot3D(x, y, z, 'red')

# ax.plot3D(x[1],y[1],z[1], '-o', markersize=10,markerfacecolor='red')
# ax.set_xlabel("X")
# ax.set_ylabel("Y")
# ax.set_zlabel("Z")
# ax.set_box_aspect((np.ptp(x), np.ptp(y), np.ptp(z)))
# p.savefig(savefile)
# p.show()

# print("Load a ply point cloud, print it, and render it")
# # ply_point_cloud = o3d.data.PLYPointCloud()
# pcd = o3d.io.read_point_cloud(pcdname)
# print(pcd)
# print(np.asarray(pcd.points))


# line_set = o3d.geometry.LineSet()
# colors = [[1, 0, 0] for i in range(len(x))]

# points = [x,y,z]
# lines = [list(range(0, len(x)-1)),list(range(1, len(x)))]
# line_set.lines = o3d.utility.Vector2iVector(np.transpose(lines))
# line_set.points = o3d.utility.Vector3dVector(np.transpose(points))
# line_set.colors = o3d.utility.Vector3dVector(colors)
# o3d.visualization.draw_geometries([pcd, line_set],
#                                   zoom=0.3412,
#                                   front=[0.4257, -0.2125, -0.8795],
#                                   lookat=[2.6172, 2.0475, 1.532],
#                                   up=[-0.0694, -0.9768, 0.2024])

