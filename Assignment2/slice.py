import pylas
import numpy as np

las = pylas.read("Assignment02_filtered.las")

minx = min(las.x)
maxx = max(las.x)
miny = min(las.y)
maxy = max(las.y)
minz = min(las.z)
maxz = max(las.z)

print(minx)
print(maxx)
print(miny)
print(maxy)
print(minz)
print(maxz)


# Max x: 73.4
# Min x: -12.87
# Max y: 10.52
# Min y: -58.81
# Max z: 21.0981
minz = -2.1
new_file = pylas.create_from_header(las.header)

for i in range(0, 4):
    new_file.points = las.points[((las.z>minz+i/2) & (las.z<minz+(i/2)+0.5))]
    print(new_file.header.point_count)
    new_file.write("slice_{}.las".format(i))
