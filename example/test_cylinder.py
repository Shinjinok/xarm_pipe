import copy
import sys

import numpy as np
import open3d as o3d

sys.path.append(".")
import pyransac3d as pyrsc
import flattten_pcd as fp
import matplotlib.pyplot as plt



pcd1 = o3d.io.read_point_cloud("./point_pipe.pcd")
o3d.visualization.draw_geometries([pcd1],width=500, height=500,left=1000, top=1280)

eq, inliers = fp.get_plane(pcd1,thresh=0.1,maxIteration=100)
print(eq)
print(inliers)
#o3d.visualization.draw_geometries([pcd2],width=500, height=500,left=1000, top=1280)

#pcd2 = fp.get_flattened_pcds2(source=pcd1,A=0,B=1,C=0,D=0,x0=0,y0=1000,z0=0)
pcd = pcd1
plane_model, inliers = pcd.segment_plane(distance_threshold=0.01,
                                         ransac_n=3,
                                         num_iterations=1000)
[a, b, c, d] = plane_model
print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

inlier_cloud = pcd.select_by_index(inliers)
inlier_cloud.paint_uniform_color([1.0, 0, 0])
outlier_cloud = pcd.select_by_index(inliers, invert=True)
o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud],
                                  zoom=0.8,
                                  front=[-0.4999, -0.1659, -0.8499],
                                  lookat=[2.1813, 2.0619, 2.0999],
                                  up=[0.1204, -0.9852, 0.1215])

with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
    labels = np.array(pcd.cluster_dbscan(eps=0.1, min_points=10, print_progress=True))

max_label = labels.max()
print(f"point cloud has {max_label + 1} clusters")
colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
colors[labels < 0] = 0
pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
o3d.visualization.draw_geometries([pcd],
                                  zoom=0.455,
                                  front=[-0.4999, -0.1659, -0.8499],
                                  lookat=[2.1813, 2.0619, 2.0999],
                                  up=[0.1204, -0.9852, 0.1215])



#o3d.visualization.draw_geometries([pcd1(inliers)],width=500, height=500,left=1000, top=1280)



center, normal, radius, inliers = fp.get_cylinder(pcd2, thresh=0.1, maxIteration=1)
mesh_cylinder = fp.get_clylinder_mesh(pcd2,center, normal, radius, inliers)
print("center: " + str(center))
print("radius: " + str(radius))
print("vecC: " + str(normal))
print("inliers: ", inliers)

o3d.visualization.draw_geometries([mesh_cylinder,pcd1],width=500, height=500,left=1000, top=1280)
