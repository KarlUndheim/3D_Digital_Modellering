import pylas
import numpy as np
import open3d as o3d
import time
import math
from numpy.linalg import eig
from functions import findCentroid, covMatrix
import pickle

with open("FINAL_v2_labels.pickle", "rb") as f:
    dict = pickle.load(f)
f.close()

labelCount = len(list(set(dict.values())))
print(labelCount)

pcd = pylas.read("Datafinalproject.las")

minx = np.min(pcd.x)
miny = np.min(pcd.y)
minz = np.min(pcd.z)

temp_pcd = pylas.create_from_header(pcd.header)

arr = []

for key, val in dict.items():
    keys = [int(k) for k in key.split(".")]
    arr.append([keys, val])
arr.sort(key=lambda x: x[0][1])

v_size = 2.0

labels2classes = [2, 0, 5]

boundingMinx = minx-v_size/2
boundingMiny = miny-v_size/2
boundingMinz = minz-v_size/2

def createPointclouds(labels, segments, ylength):
    for label in range(labels):
        currentClass = labels2classes[label]
        index = 0
        currentVox = 0
        voxels = [key for key,value in arr if value==label]
        new_pcds = []
        segLength = ylength/segments
        for segment in range(segments+1):
            print(segment)
            temp_pcd.points = pcd.points[((pcd.y>=(boundingMiny+segLength*segment)) & (pcd.y<=(boundingMiny+segLength*(segment+1))))]
                
            for vox in voxels[currentVox:]:
                
                i = vox[0]
                j = vox[1]
                k = vox[2]

                if j>=((segLength/v_size)*segment + segLength/v_size):
                    currentVox = index
                    break

                new_pcd = pylas.create_from_header(pcd.header)
                new_pcd.points = temp_pcd.points[((temp_pcd.x>=boundingMinx+v_size*i) & (temp_pcd.x<=boundingMinx+v_size*(i+1)) & (temp_pcd.y>=boundingMiny+v_size*j) & (temp_pcd.y<=boundingMiny+v_size*(j+1)) & (temp_pcd.z>=boundingMinz+v_size*k) & (temp_pcd.z<=boundingMinz+v_size*(k+1)))]
                new_pcd.user_data[True] = currentClass
                new_pcds.append(new_pcd)
                index+=1
                
        las = pylas.merge([p for p in new_pcds])
        las.write("Lable_v2_FINAL_{}.las".format(label))

createPointclouds(labelCount, 1200, 2400)

