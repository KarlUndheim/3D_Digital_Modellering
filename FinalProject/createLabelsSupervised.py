import pylas
import numpy as np
import open3d as o3d
import time
import math
from numpy.linalg import eig
from functions import findCentroid, covMatrix
import pickle
import random

with open("FINAL_v2_features.pickle", "rb") as f:
    loaded_dict = pickle.load(f)

road = 0
building = 1
tree = 2

labelDict = {}

for key, value in loaded_dict.items():
    planarity = value[0][0]
    scattering = value[1][0]
    omnivariance = value[2][0]
    anisotropy = value[3][0]
    sumeig = value[4][0]
    curv = value[5][0]
    verticality = value[6][0]
    linearity = value[7][0]

    if verticality>0.98:
        if curv>0.01:
            labelDict[key] = building
        else:
            labelDict[key] = road
        continue

    if scattering>0.2 or omnivariance>0.2:
        labelDict[key] = tree
        continue

    if scattering>0.08 or omnivariance>0.09:
        if (planarity>0.55 or linearity > 0.8):
            labelDict[key] = building
            continue
        else:
            labelDict[key] = tree
        continue
    else:
        if planarity < 0.3 and linearity<0.8:
            labelDict[key] = tree
        else:
            labelDict[key] = building
    


print(len(labelDict))
with open("FINAL_v2_labels.pickle", "wb") as f:
    pickle.dump(labelDict, f)
f.close()
