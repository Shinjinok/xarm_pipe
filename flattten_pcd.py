
import numpy as np
import pyransac3d as pyrsc
import open3d as o3d
import copy
from tf import transformations
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3, Polygon


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

def get_cylinder(pcd2,thresh=0.1,maxIteration=1000):
    cil = pyrsc.Cylinder()
    points = np.asarray(pcd2.points)
    center, normal, radius, inliers = cil.fit(points, thresh, maxIteration)
    return center, normal, radius, inliers

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