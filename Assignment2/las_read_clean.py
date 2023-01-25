# import open3d as o3d
# from pyntcloud import PyntCloud

import numpy as np
from scipy import stats
import pylas


print("Load a ply point cloud, print it, and render it")
#### load by pylas
pcd = pylas.read("Assignment02.las")

#### create save file
new_file = pylas.create(point_format_id=2, file_version="1.2") #(point_format_id=pcd.header.point_format_id, file_version=pcd.header.version)

# choose points with z value more than the mode
new_file.points = pcd.points[((pcd.x > -50) & (pcd.x<10) & (pcd.y>-50) & (pcd.y<10))]

# print(pcd.points)
# print(new_file.x)
#### save las file
print(new_file.header.point_count)
new_file.write("Assignment02_clean.las")
