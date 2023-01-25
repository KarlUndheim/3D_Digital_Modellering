import pylas
import numpy as np
import open3d as o3d
import time
import math
from numpy.linalg import eig
from functions import findCentroid, covMatrix
import pickle

start_time = time.time()
pcd = pylas.read("Datafinalproject.las")
minx = min(pcd.x)
miny = min(pcd.y)
minz = min(pcd.z)

maxx = max(pcd.x)
maxy = max(pcd.y)
maxz = max(pcd.z)

print(minx, maxx, miny, maxy, minz, maxz)

o3d_pcd = o3d.geometry.PointCloud()
o3d_pcd.points = o3d.utility.Vector3dVector(np.vstack((pcd.x, pcd.y, pcd.z)).transpose())
v_size=2.0
voxel_grid=o3d.geometry.VoxelGrid.create_from_point_cloud(o3d_pcd,voxel_size=v_size)
voxels = voxel_grid.get_voxels()

print("Time for voxelization: ", time.time() - start_time, " s")

ijk = []
for v in voxels:
    i,j,k = v.grid_index
    ijk.append([i,j,k])
ijk.sort(key=lambda x: x[1])

print(ijk[0:10])

print(voxel_grid.get_axis_aligned_bounding_box())
print(voxel_grid.voxel_size)


def createFeatureDictionary(segments, ylength, outFileName):

    new_pcd = pylas.create_from_header(pcd.header)
    currentVox = 0
    index = 0
    dict = {}
    segLength = ylength/segments
    boundingMinx = minx-v_size/2
    boundingMiny = miny-v_size/2
    boundingMinz = minz-v_size/2
    removed = 0

    for segment in range(segments+1):
        print(segment)
        new_pcd.points = pcd.points[((pcd.y>=(boundingMiny+segLength*segment)) & (pcd.y<=(boundingMiny+segLength*(segment+1))))]
        for vox in ijk[currentVox:]:
            
            i = vox[0]
            j = vox[1]
            k = vox[2]

            if j>=((segLength/v_size)*(segment+1)):
                currentVox = index
                break

            vox_pcd = pylas.create_from_header(pcd.header)
            vox_pcd.points = new_pcd.points[((new_pcd.x>=(boundingMinx+v_size*i)) & (new_pcd.x<=(boundingMinx+v_size*(i+1))) & (new_pcd.y>=(boundingMiny+v_size*j)) & (new_pcd.y<=(boundingMiny+v_size*(j+1))) & (new_pcd.z>=(boundingMinz+v_size*k)) & (new_pcd.z<=(boundingMinz+v_size*(k+1))))]
            if vox_pcd.header.point_count<3:
                index+=1
                removed+=1
                continue
            dict[str(i)+"."+str(j)+"."+str(k)] = vox_pcd
            index+=1
        print("My program took", time.time() - start_time, " s to run")
        print(index)

    """ newCloud = pylas.merge([cloud for cloud in dict.values()])
    newCloud.write("Glued.las") """

    print("Removed: ", removed)
    new_dict = {}

    error = 0
    for index, neighborhood in dict.items():
        cov = covMatrix(neighborhood)
        eigs = eig(cov)
        eigvals = eigs[0]
        eigVectors = eigs[1]

        verticality = np.max([abs(vec) for vec in eigVectors])

        eigvals.sort()

        eigvals = eigvals[::-1]
        e1 = eigvals[0]
        e2 = eigvals[1]
        e3 = eigvals[2]

        if e1==0:
            print("NULL HER")
            continue

        linearity = (e1-e2)/e1
        planarity = (e2-e3)/e1
        scattering = e3/e1
        anisotropy = (e1-e3)/e1
        try:
            omnivariance = np.cbrt(e1*e2*e3)
        except:
            error+=1
            continue
        sumeig = e1+e2+e3
        curv = e3/(e1+e2+e3)
        verticality = np.max([abs(v) for v in eigVectors[2]])
        """ try:
            eigenentropy = -(e1*math.log(e1) + e2*math.log(e2) + e3*math.log(e3))
        except:
            error+=1
            continue """
         
        vec = np.vstack([planarity, scattering, omnivariance, anisotropy, sumeig, curv, verticality, linearity])
        new_dict[index] = vec
    print("Error: ", error)

    with open("{}.pickle".format(outFileName), "wb") as f:
        pickle.dump(new_dict, f)

createFeatureDictionary(1200, 2400, "FINAL_v2_features")

print("My program took", time.time() - start_time, " s to run")






