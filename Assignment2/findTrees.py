import math

centres = []

locatedTrees = []

for i in range(4):
    slice = []
    with open("centres_new_{}.txt".format(i), "r") as f:
        for circle in f:
            circle = [float(coord) for coord in circle.split()]
            slice.append((circle))
    centres.append(slice)
    f.close()

for sliceNr, slice in enumerate(centres[:2]):
    for core_center in slice:
        if core_center[0]==0:
            continue
        potentialTree = []
        deleteIndexes = []
        potentialTree.append(core_center)
        for i in range(sliceNr+1, 4):
            for j, align_center in enumerate(centres[i]):
                if (math.sqrt((core_center[0]-align_center[0])**2 + (core_center[1]-align_center[1])**2)<1.0):
                    potentialTree.append(align_center)
                    deleteIndexes.append([sliceNr+1,j])
        if len(potentialTree)>=3:
            locatedTrees.append(core_center)
            for delete in deleteIndexes:
                centres[delete[0]][delete[1]]=[0,0]

for tree in locatedTrees:
    print(tree)

with open("trees_new.txt", "w") as trees:
    for tree in locatedTrees:
        trees.write(str(tree[0])[:6]+" "+str(tree[1])[:6]+" "+str(tree[2])[:6]+"\n")
trees.close()