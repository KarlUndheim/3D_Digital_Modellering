import pylas

# mk5: Drit bra: 3,7,8,9,10,11,13,18,22,23,24(imponerende),27(clean),29,30,31,37,39(imponerende),47,50(vanskelig tak),51,58
best20 = [3,7,8,9,10,13,18,22,23,24,27,29,30,31,37,39,47,50,51,58]
pcd = pylas.read("Data29_clean.las")

las = pylas.create_from_header(pcd.header)

# combine the two files
las = pylas.merge([pylas.read("Data29_roofmk6_{}.las".format(i)) for i in range(63)])

las.write("Data29_roofmk6combined.las")



