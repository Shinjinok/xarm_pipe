
import numpy as np
import pyransac3d as pyrsc
import open3d as o3d
import copy
from tf import transformations
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3, Polygon
import matplotlib.pyplot as plt

MaxX = 1.5
MinX = 0
MaxZ = 1.5
MinZ = 0
MaxR = 1

def get_flattened_pcds2(source,A,B,C,D,x0,y0,z0):
    x1 = np.asarray(source.points)[:,0]
    y1 = np.asarray(source.points)[:,1]
    z1 = np.asarray(source.points)[:,2]
    x0 = x0 * np.ones(x1.size)
    y0 = y0 * np.ones(y1.size)
    z0 = z0 * np.ones(z1.size)
    r = np.power(np.square(x1-x0)+np.square(y1-y0)+np.square(z1-z0),0.5)
    a = (x1-x0)/r
    b = (y1-y0)/r
    c = (z1-z0)/r
    t = -1 * (A * np.asarray(source.points)[:,0] + B * np.asarray(source.points)[:,1] + C * np.asarray(source.points)[:,2] + D)
    t = t / (a*A+b*B+c*C)
    np.asarray(source.points)[:,0] = x1 + a * t
    np.asarray(source.points)[:,1] = y1 + b * t
    np.asarray(source.points)[:,2] = z1 + c * t
    return source

def get_cylinder(pcd,thresh=0.1,maxIteration=1000):
    print("length pcd ", len(np.array(pcd.points)))
    #o3d.visualization.draw_geometries([load_pcd])
    voxel_size=0.05
    print("Downsample the point cloud with a voxel of ", voxel_size)
    pcd2 = pcd.voxel_down_sample(voxel_size)
    #print("length down sampling pcd ", len(np.array(pcd2.points)))
    #o3d.visualization.draw_geometries([pcd2])
    plane_model, inliers = pcd2.segment_plane(distance_threshold=0.1,
                                         ransac_n=3,
                                         num_iterations=1000)
    [a, b, c, d] = plane_model
    print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

    inlier_cloud = pcd2.select_by_index(inliers)
    #print(inliers)
    inlier_cloud.paint_uniform_color([1.0, 0, 0])
    outlier_cloud = pcd2.select_by_index(inliers, invert=True)
    outlier_cloud.paint_uniform_color([0, 1.0, 0])

    if inlier_cloud == np.NaN:
        outlier_cloud = pcd2
    #print(len(inliers), len(np.array(pcd2.points)))
    #print("inlider")
#    o3d.visualization.draw_geometries([inlier_cloud])
    #print("outlier_cloud")
#    o3d.visualization.draw_geometries([outlier_cloud])
    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
        labels = np.array(outlier_cloud.cluster_dbscan(eps=0.1, min_points=10, print_progress=True))
    max_label = labels.max()
    print(f"point cloud has {max_label + 1} clusters")
    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0
    outlier_cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])
    #print("length label:  ",len(labels))
    #print("length outlier_cloud:  ",len(np.array(outlier_cloud.points)))
    center = []
    normal = []
    radius = []
    
    for c in range(max_label+1):
        print(c)
        cluster = [i for i, n in enumerate(labels.tolist()) if n == c]
        #print(cluster) 
        cluster = outlier_cloud.select_by_index(cluster)
        #print("length cluster:  ",len(np.array(cluster.points)))
        #o3d.visualization.draw_geometries([cluster])
    #o3d.visualization.draw_geometries([outlier_cloud])
        cil = pyrsc.Cylinder()
        points = np.asarray(cluster.points)
        cen, nor, rad, inliers = cil.fit(points, thresh, maxIteration)
        print("center: " + str(cen))
        print("radius: " + str(rad))
        print("vecC: " + str(nor))
        if (rad < MaxR and cen[0] > MinX and cen[0] < MaxX and cen[2] > MinZ and cen[2] < MaxZ):
            center.append(cen)
            normal.append(nor)
            radius.append(rad)

    
        
    
    
    return center, normal, radius

def get_clylinder_mesh(center, normal, radius):

    R = pyrsc.get_rotationMatrix_from_vectors([0, 0, 1], normal)
    mesh_cylinder = o3d.geometry.TriangleMesh.create_cylinder(radius=radius, height=0.5)
    mesh_cylinder.compute_vertex_normals()
    mesh_cylinder.paint_uniform_color([0.1, 0.9, 0.1])
    mesh_cylinder = mesh_cylinder.rotate(R, center=[0, 0, 0])
    mesh_cylinder = mesh_cylinder.translate((center[0], center[1], center[2]))
    return mesh_cylinder

def get_clylinder_pos(center, normal):
    r = pyrsc.get_rotationMatrix_from_vectors([0, 0, 1], normal)
    R = np.identity(4)
    R[0:3,0:3] = r
    print("R :",np.array(R))
    print(len(R))
    qt = transformations.quaternion_from_matrix(R)
    print("qt :",qt)
    P = Pose(Point(center[0],center[1],center[2]),Quaternion(qt[0],qt[1],qt[2],qt[3]))
    #p = [center[0],center[1],center[2]]
    #q = [qt[0],qt[1],qt[2],qt[3]]
    return P

def get_plane(pcd2,thresh=0.1,maxIteration=1000):
    cil = pyrsc.Plane()
    points = np.asarray(pcd2.points)
    eq, inliers = cil.fit(points, thresh, maxIteration)
    return eq, inliers
#R = pyrsc.get_otationMatrix_from_vectors([0, 0, 1], normal)

#plane = pcd2.select_by_index(inliers).paint_uniform_color([1, 0, 0])
# obb = plane.get_oriented_bounding_box()
# obb2 = plane.get_axis_aligned_bounding_box()
# obb.color = [0, 0, 1]
# obb2.color = [0, 1, 0]
#not_plane = pcd2.select_by_index(inliers, invert=True)

#mesh = o3d.geometry.create_mesh_cylinder(radius, 0.5,resolution = 20, split=4)
#mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(origin=[0, 0, 0], size=0.2)
#cen = o3d.geometry.TriangleMesh.create_coordinate_frame(origin=center, size=0.5)
#mesh_rot = copy.deepcopy(mesh).rotate(R, center=[0, 0, 0])

#mesh_cylinder = o3d.geometry.TriangleMesh.create_cylinder(radius=radius, height=0.5)
#mesh_cylinder.compute_vertex_normals()
#mesh_cylinder.paint_uniform_color([0.1, 0.9, 0.1])
#mesh_cylinder = mesh_cylinder.rotate(R, center=[0, 0, 0])
#mesh_cylinder = mesh_cylinder.translate((center[0], center[1], center[2]))
#o3d.visualization.draw_geometries([mesh_cylinder])