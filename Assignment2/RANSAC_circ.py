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
        self.pcd_p = np.array([self.pcd.x, self.pcd.y]) # 3* n_points
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
        best_circle = []

        for i in range(self.Max_Iter):
            print('=======================iteration no. ', i, '=======================')
            # Step 1: add 3 random points
            random.seed()

            TooFewPoints = True

            while TooFewPoints:

                start_points_idx = [random.randint(0, self.point_count-1) for i_l in range(3)] # start_points = np.random.randint(0, self.point_count, size=3)

                start_idx = start_points_idx[0]


                start_x = self.pcd.x[start_idx]
                start_y = self.pcd.y[start_idx]

                new_file_temp = pylas.create_from_header(self.pcd.header)
                new_file_temp.points = self.pcd.points[((self.pcd.x > start_x-0.5) & (self.pcd.x < start_x+0.5) & (self.pcd.y < start_y+0.5) & (self.pcd.y > start_y-0.5))]
                
                if(new_file_temp.header.point_count<=3):
                    continue
                    
                TooFewPoints=False

                samples = random.sample(range(0, new_file_temp.header.point_count-1), 3)

                first_point = samples[0]
                second_point = samples[1]
                third_point = samples[2]

            x1 = new_file_temp.x[first_point]
            x2 = new_file_temp.x[second_point]
            x3 = new_file_temp.x[third_point]
            y1 = new_file_temp.y[first_point]
            y2 = new_file_temp.y[second_point]
            y3 = new_file_temp.y[third_point]

            print('start_points_idx: ', start_points_idx)
            p1 = [x1, y1]
            p2 = [x2, y2]
            p3 = [x3, y3]
            print('3 random points:\np1: {}\np2: {}\np3: {}\n'.format(p1, p2, p3))

            c = (x1-x2)**2 + (y1-y2)**2
            a = (x2-x3)**2 + (y2-y3)**2
            b = (x3-x1)**2 + (y3-y1)**2
            s = 2*(a*b + b*c + c*a) - (a*a + b*b + c*c) 
            x_c = (a*(b+c-a)*x1 + b*(c+a-b)*x2 + c*(a+b-c)*x3) / s
            y_c = (a*(b+c-a)*y1 + b*(c+a-b)*y2 + c*(a+b-c)*y3) / s 
            ar = a**0.5
            br = b**0.5
            cr = c**0.5 
            r = ar*br*cr / ((ar+br+cr)*(-ar+br+cr)*(ar-br+cr)*(ar+br-cr))**0.5

            # Step 3: add points into interior points
            # Skip points have be chosen as seed
            # pcd_p_search = np.delete(self.pcd_p, start_points_idx, axis=0)

            #  Calculate distance between the point and the plane
            #  (now all point are in calculating, including 3 samples)

            new_file_p = np.array([new_file_temp.x, new_file_temp.y])
            new_file_p = new_file_p.T

            dist = np.array([np.abs(math.sqrt((new_file_temp.x[i] - x_c)**2 + (new_file_temp.y[i] - y_c)**2)-r) for i in range(0, new_file_temp.header.point_count)])

            # Add points in distance threshold as interior points

            interior_idx = np.where((dist <= self.thres_dis))[0]

            """ print('interior max dist: {} : {}, min dist: {} : {}'.format(
                max(dist), max(dist[interior_idx]), min(dist[interior_idx]), min(dist)))
            print(interior_idx.shape) """
            
            pcd_p_interior = new_file_p[interior_idx, :]

            # update the best model
            if (pcd_p_interior.shape[0] > best_interior.shape[0]) and (pcd_p_interior.shape[0]< self.point_count): # and (pcd_p_interior.shape[0]< self.point_count/2 * 1.15):
                print('u = pdating...')
                root_idx = start_idx
                best_interior = pcd_p_interior
                best_interior_idx = interior_idx
                best_circle = [x_c, y_c, r]

        return self.pcd, best_interior, best_interior_idx, best_circle, root_idx



if __name__=='__main__':
    ## read point clouds
    for s in range(4):

        pcd = pylas.read("slice_{}.las".format(s)) # Data01-cleanheight-1-roofblock.las: pcd.header.point_count =72590
        print('statistic of pcd: max z={}, min z={}, diff={}'.format(max(pcd.z), min(pcd.z), (max(pcd.z)-min(pcd.z))))
        # print(pcd)
        i=0
        all_point_count = pcd.header.point_count
        count_threshold = all_point_count / 3

        centres =  open("centres_new_{}.txt".format(s), "w")

        while pcd.header.point_count > count_threshold:
            rans = RANSAC(pcd, 100, 0.2)
            pcd_new, best_interior, best_interior_idx, best_circle, root_idx = rans.RanSac_algthm()
            print('input count: {}, output count: {}'.format(pcd.header.point_count, best_interior_idx.shape))

            # save result
            
            start_x = pcd.x[root_idx]
            start_y = pcd.y[root_idx]

            """ new_file = pylas.create_from_header(pcd.header)
            new_file.points = pcd.points[((pcd.x > start_x-0.5) & (pcd.x < start_x+0.5) & (pcd.y < start_y+0.5) & (pcd.y > start_y-0.5))]
            new_file.points = new_file.points[best_interior_idx]
            new_file.write("A2_{}.las".format(i)) """

            centres.write(str(best_circle[0])+" "+str(best_circle[1])+" "+str(best_circle[2])+"\n")

            print(best_circle[0])
            print(best_circle[1])
            print(best_circle[2])

            # update point clouds

            areaSearched = np.argwhere(((pcd.x > start_x-0.5) & (pcd.x < start_x+0.5) & (pcd.y < start_y+0.5) & (pcd.y > start_y-0.5)))

            pcd.points = np.delete(pcd.points, areaSearched)#delete the points have already be detected as an plane
            print(pcd.header.point_count)
            i += 1
        centres.close()



