import pylas
import numpy as np
import math

las = pylas.read("Assignment02_filtered.las")

with open("trees_new.txt", "r") as f:
    i = 0
    for line in f:
        x,y,r = [float(j) for j in line.split()]

        new_file = pylas.create_from_header(las.header)
        new_file.points = las.points[((las.x>x-2) & (las.x<x+2) & (las.y>y-2) & (las.y<y+2))]
        new_file.write("tree_new_{}.las".format(i))
        i+=1
f.close()