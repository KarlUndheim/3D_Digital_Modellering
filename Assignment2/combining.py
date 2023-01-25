import numpy as np
import pylas
#from mpl_toolkits.mplot3d import Axes3D
#import matplotlib.pyplot as plt
import open3d as o3d

las1 = pylas.read("Data29_roo_0.las")
las2 = pylas.read("Data29_roo_1.las")
las3 = pylas.read("Data29_roo_2.las")

las = pylas.create(point_format_id=2, file_version="1.2")

with pylas.open("Data29_roo_0.las") as input_las:
    with pylas.open("Data29_roo_1.las","a") as ground_las:
        for points in input_las.read():
            ground_las.(points)



point_data = np.stack([ground_las.X, ground_las.Y, ground_las.Z], axis=0).transpose((1, 0))

geom = o3d.geometry.PointCloud()
geom.points = o3d.utility.Vector3dVector(point_data)
o3d.visualization.draw_geometries([geom])