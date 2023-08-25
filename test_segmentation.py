import copy
import sys

import numpy as np
import open3d as o3d

sys.path.append(".")
import pyransac3d as pyrsc
import flattten_pcd as fp
import matplotlib.pyplot as plt



load_pcd = o3d.io.read_point_cloud("point_pipe.pcd")
print("length pcd ", len(np.array(load_pcd.points)))
o3d.visualization.draw_geometries([load_pcd])

#pcd2 = pcd.remove_duplicated_points()
#print("length pcd2 ", len(np.array(pcd2.points)))
"""
print("Downsample the point cloud with a voxel of 0.05")
pcd2 = pcd.voxel_down_sample(voxel_size=0.05)
print("length downpcd ", len(np.array(pcd2.points)))
o3d.visualization.draw_geometries([pcd2])
"""
plane_model, inliers = load_pcd.segment_plane(distance_threshold=0.05,
                                         ransac_n=3,
                                         num_iterations=1000)
[a, b, c, d] = plane_model
print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

inlier_cloud = load_pcd.select_by_index(inliers)
inlier_cloud.paint_uniform_color([1.0, 0, 0])
outlier_cloud = load_pcd.select_by_index(inliers, invert=True)
outlier_cloud.paint_uniform_color([0, 1.0, 0])

print("inlider")
o3d.visualization.draw_geometries([inlier_cloud])
print("outlier_cloud")
o3d.visualization.draw_geometries([outlier_cloud])

with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
    labels = np.array(outlier_cloud.cluster_dbscan(eps=0.1, min_points=100, print_progress=True))

max_label = labels.max()
print(f"point cloud has {max_label + 1} clusters")
colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
colors[labels < 0] = 0
outlier_cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])

#print("length label:  ",len(labels))
#print("length outlier_cloud:  ",len(np.array(outlier_cloud.points)))
o3d.visualization.draw_geometries([outlier_cloud])

cluster0_list = labels.tolist()
cluster = outlier_cloud.select_by_index(cluster0_list)
print("length cluster:  ",len(np.array(cluster.points)))
o3d.visualization.draw_geometries([cluster])


