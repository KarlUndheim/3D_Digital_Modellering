import numpy as np
import pylas
import random
import math
# import pandas as pd


class RANSAC:
    """
    RANSAC algorithm
        INPUT:
            pcd:            np.array()     # point clouds data
            MAX_Iteration:  int            # the number you want for max iterations
            Thres_dis:      int            # distance threshold to
        OUTPUT:
            self.pcd
            best_interior:      np.array()
            best_interior_idx:  list
            best_plane:         list
    """
    def __init__(self, pcd, MAX_Iteration, Thres_dis):
        self.pcd = pcd
        self.point_count = self.pcd.header.point_count
        self.pcd_p = np.array([self.pcd.x, self.pcd.y, self.pcd.z]) # 3* n_points
        self.pcd_p = self.pcd_p.T   # n_points* 3
        print('point clouds size: ', self.pcd_p.shape)
        self.Max_Iter = MAX_Iteration
        self.thres_dis = Thres_dis

    def RanSac_algthm(self):
        """
        INPUT: self.pcd;    self.Max_Iter;  self.thres_dis
        :return: self.pcd, best_interior, best_interior_idx, best_plane
        """
        best_interior = np.array([])
        best_interior_idx = np.array([])
        best_plane = []

        class_number = 10

        updateIndexes = []

        for i in range(self.Max_Iter):
            print('=======================iteration no. ', i, '=======================')
            # Step 1: add 3 random points
            random.seed()

            TooFewPoints = True

            while TooFewPoints:

                start_points_idx = [random.randint(0, self.point_count-1) for i_l in range(3)] # start_points = np.random.randint(0, self.point_count, size=3)

                start_idx = start_points_idx[0]
                start_x = self.pcd.x[start_points_idx[0]]
                start_y = self.pcd.y[start_points_idx[0]]
                start_z = self.pcd.z[start_points_idx[0]]

                new_file_temp = pylas.create(point_format_id=2, file_version="1.2")
                new_file_temp.points = self.pcd.points[((self.pcd.x > start_x-1.0) & (self.pcd.x < start_x+1.0) & (self.pcd.y < start_y+1.0) & (self.pcd.y > start_y-1.0) & (self.pcd.z < start_z+1.0) & (self.pcd.z > start_z-1.0))]
                
                if(new_file_temp.header.point_count<=3):
                    continue
                    
                TooFewPoints=False

                samples = random.sample(range(0, new_file_temp.header.point_count-1), 3)

                first_point = samples[0]
                second_point = samples[1]
                third_point = samples[2]

                index_1 = np.where((self.pcd.x==new_file_temp.x[first_point]) & (self.pcd.y == new_file_temp.y[first_point]))[0][0]
                index_2 = np.where((self.pcd.x==new_file_temp.x[second_point]) & (self.pcd.y == new_file_temp.y[second_point]))[0][0]
                index_3 = np.where((self.pcd.x==new_file_temp.x[third_point]) & (self.pcd.y == new_file_temp.y[third_point]))[0][0]
                
                start_points_idx[0]=index_1
                start_points_idx[1]=index_2
                start_points_idx[2]=index_3

                z1 = self.pcd.z[index_1]
                z2 = self.pcd.z[index_2]
                z3 = self.pcd.z[index_3]

                if((abs(z1-z2)<0.5) & (abs(z1-z3)<0.5)):
                    TooFewPoints=True

            """ print('start_points_idx: ', start_points_idx) """
            p1 = [self.pcd.x[start_points_idx[0]], self.pcd.y[start_points_idx[0]], self.pcd.z[start_points_idx[0]]]
            p2 = [self.pcd.x[start_points_idx[1]], self.pcd.y[start_points_idx[1]], self.pcd.z[start_points_idx[1]]]
            p3 = [self.pcd.x[start_points_idx[2]], self.pcd.y[start_points_idx[2]], self.pcd.z[start_points_idx[2]]]
            print('3 random points:\np1: {}\np2: {}\np3: {}\n'.format(p1, p2, p3))

            # Step 2: Calculate plane contains p1, p2 and p3
            # Plane equation: ax + by + cz + d = 0
            # p1: x1, y1, z1
            # p2: x2, y2, z2
            # p3: x3, y3, z3
            a = (p2[1] - p1[1]) * (p3[2] - p1[2]) - (p2[2] - p1[2]) * (p3[1] - p1[1]) # a = (y2 - y1)*(z3 - z1) - (z2 - z1)*(y3 - y1)
            b = (p2[2] - p1[2]) * (p3[0] - p1[0]) - (p2[0] - p1[0]) * (p3[2] - p1[2]) # b = (z2 - z1)*(x3 - x1) - (x2 - x1)*(z3 - z1)
            c = (p2[0] - p1[0]) * (p3[1] - p1[1]) - (p2[1] - p1[1]) * (p3[0] - p1[0]) # c = (x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1)
            d = -(a * p1[0] + b * p1[1] + c * p1[2])# d = -(a*x1 + b*y1 + c*z1)
            plane_len = max(0.01, math.sqrt(a*a + b*b + c*c))

            # Step 3: add points into interior points
            # Skip points have be chosen as seed
            # pcd_p_search = np.delete(self.pcd_p, start_points_idx, axis=0)

            #  Calculate distance between the point and the plane
            #  (now all point are in calculating, including 3 samples)
            dist = np.abs((a*self.pcd_p[:, 0] + b * self.pcd_p[:, 1] + c * self.pcd_p[:, 2] + d)) / plane_len
            """ print(dist[0], dist[start_points_idx[0]], dist[start_points_idx[1]], dist[start_points_idx[2]]) """
            """ print('distance from every point to this plane: ', dist) """

            # Add points in distance threshold as interior points

            interior_idx_point = np.where((abs(self.pcd.x - start_x)<15) & (abs(self.pcd.y - start_y)<15) & (abs(self.pcd.z - start_z)<5))[0]
            interior_idx_plane = np.where((dist <= self.thres_dis))[0]

            interior_idx = np.intersect1d(interior_idx_point, interior_idx_plane)


            """ print('interior max dist: {} : {}, min dist: {} : {}'.format(
                max(dist), max(dist[interior_idx]), min(dist[interior_idx]), min(dist)))
            print(interior_idx.shape) """
            pcd_p_interior = self.pcd_p[interior_idx, :]

            # update the best model
            if (pcd_p_interior.shape[0] > best_interior.shape[0]) and (pcd_p_interior.shape[0]< self.point_count): # and (pcd_p_interior.shape[0]< self.point_count/2 * 1.15):
                print('updating...')
                updateIndexes.append(i)
                best_interior = pcd_p_interior
                best_interior_idx = interior_idx
                best_plane = [a, b, c, d, plane_len]
                self.pcd.classification[interior_idx] = self.pcd.classification[interior_idx] + class_number
        print(updateIndexes)



        return self.pcd, best_interior, best_interior_idx, best_plane, updateIndexes[-1]



if __name__=='__main__':
    ## read point clouds
    pcd = pylas.read("Data29_clean.las") # Data01-cleanheight-1-roofblock.las: pcd.header.point_count =72590
    print('statistic of pcd: max z={}, min z={}, diff={}'.format(max(pcd.z), min(pcd.z), (max(pcd.z)-min(pcd.z))))
    # print(pcd)
    i=0
    all_point_count = pcd.header.point_count
    count_threshold = all_point_count / 10

    f = open("IterationsUsed.txt", "w")

    while pcd.header.point_count > count_threshold:
        rans = RANSAC(pcd, 100, 0.4)
        pcd_new, best_interior, best_interior_idx, best_plane, iterationsUsed = rans.RanSac_algthm()
        print('input count: {}, output count: {}'.format(pcd.header.point_count, best_interior_idx.shape))

        
        f.write("Plane nr: {}  Iterations used: {}".format(i,iterationsUsed)+"\n")

        # save result
        new_file = pylas.create_from_header(pcd.header)
        new_file.points = pcd.points[best_interior_idx]
        new_file.write("Data29_roofmk6_{}.las".format(i))

        # update point clouds
        pcd.points = np.delete(pcd.points, best_interior_idx)#delete the points have already be detected as an plane
        print(pcd.header.point_count)
        i += 1




