# import open3d as o3d
# from pyntcloud import PyntCloud

import numpy as np
from scipy import stats
import pylas


print("Load a ply point cloud, print it, and render it")
#### load by pylas
pcd = pylas.read("Data29.las")
a = pcd.points
# print(pcd.x[0], pcd.y[0], pcd.z[0])

print(pcd.header.point_format_id)

#### KARL: first I want to filter out every point classified as 13 and 14.

pcd0 = pylas.create(point_format_id=2, file_version="1.2")
pcd0.points = pcd.points[pcd.classification == 0]


#### create save file
new_file = pylas.create(point_format_id=2, file_version="1.2") #(point_format_id=pcd.header.point_format_id, file_version=pcd.header.version)


#### change height
# pcd_withheight = pcd.points
# find mode for this dataset
pcd_z_mode = stats.mode(pcd0.z)[0][0]
b = stats.mode(pcd0.z)
# print('z value max={}, min={}, mode={}'.format(max(pcd.z), min(pcd.z), pcd_z_mode))

# choose points with z value more than the mode
new_file.points = pcd0.points[pcd0.z > (pcd_z_mode+4)]

# print(pcd.points)
# print(new_file.x)
#### save las file
new_file.write("Data29_clean.las")

