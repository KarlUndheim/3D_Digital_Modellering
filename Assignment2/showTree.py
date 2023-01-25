import numpy as np
import pylas
#from mpl_toolkits.mplot3d import Axes3D
#import matplotlib.pyplot as plt
import open3d as o3d
from scipy import stats


las = pylas.read("tree_new_2.las")

point_data = np.stack([las.x, las.y, las.z], axis=0).transpose((1, 0))

""" geom = o3d.geometry.PointCloud()
geom.points = o3d.utility.Vector3dVector(point_data)
o3d.visualization.draw_geometries([geom]) """

source = o3d.geometry.PointCloud()
source.points = o3d.utility.Vector3dVector(point_data)
source.paint_uniform_color([1, 0.706, 0])
o3d.visualization.draw_geometries([source]) 