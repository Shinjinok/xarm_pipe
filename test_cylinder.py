import copy
import sys

import numpy as np
import open3d as o3d

sys.path.append(".")
import pyransac3d as pyrsc
import flattten_pcd as fp



pcd1 = o3d.io.read_point_cloud("./point_pipe.pcd")



pcd2 = fp.get_flattened_pcds2(source=pcd1,A=0,B=1,C=0,D=0,x0=0,y0=1000,z0=0)
center, normal, radius, inliers = fp.get_cylinder(pcd2, thresh=0.1, maxIteration=1)
mesh_cylinder = fp.get_clylinder_mesh(pcd2,center, normal, radius, inliers)
print("center: " + str(center))
print("radius: " + str(radius))
print("vecC: " + str(normal))
print("inliers: ", inliers)

o3d.visualization.draw_geometries([mesh_cylinder,pcd1],width=500, height=500,left=1000, top=1280)
