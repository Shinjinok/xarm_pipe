import os.path
import sys
import numpy as np
import open3d as o3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering

import time
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import pcl #sudo apt install python3-pcl
import ros_numpy #sudo apt-get install ros-noetic-ros-numpy

import pyransac3d as pyrsc #pip3 install pyransac3d
import flattten_pcd as fp

print("Project")
print("python version", sys.version)
print("open3d version", o3d.__version__)

class WindowApp:
   
    def __init__(self):

        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)
        rospy.Subscriber('/cloud2', PointCloud2, self.callback)
        self.point_cloud = o3d.geometry.PointCloud()

        self.window = gui.Application.instance.create_window("xARM6", 1400, 900)
        w = self.window

        # member variables
        self.model_dir = ""
        self.model_name = ""

        em = w.theme.font_size
        # 3D Widget
        self._widget3d = gui.SceneWidget()
        self._widget3d.scene = rendering.Open3DScene(w.renderer)
        self._widget3d.set_view_controls(gui.SceneWidget.Controls.ROTATE_CAMERA)
        # create a frame that encapsulates the Scenewidget
        self._widget3d.frame = gui.Rect(500, w.content_rect.y,
                                        900, w.content_rect.height)
        #mesh = o3d.geometry.TriangleMesh.create_sphere()
        #mesh.compute_vertex_normals()
        self.material = rendering.MaterialRecord()
        self.material.shader = "defaultLit"
        #_widget3d.scene.add_geometry('mesh', mesh, material)
        self._widget3d.scene.set_background([200, 0, 0, 200]) # not working?!
        self._widget3d.scene.camera.look_at([0, 0, 0], [1, 1, 1], [0, 0, 1])
        self._widget3d.set_on_mouse(self._on_mouse_widget3d)

        # gui layout
        gui_layout = gui.Vert(0, gui.Margins(0.5 * em, 0.5 * em, 0.5 * em, 0.5 * em))
        # create frame that encapsulates the gui
        gui_layout.frame = gui.Rect(w.content_rect.x, w.content_rect.y,
                                    500, w.content_rect.height)
        # File-chooser widget
        self._fileedit = gui.TextEdit()
        filedlgbutton = gui.Button("scan")
        filedlgbutton.horizontal_padding_em = 0.5
        filedlgbutton.vertical_padding_em = 0
        filedlgbutton.set_on_clicked(self._on_filedlg_button)


        scanbutton = gui.Button("scan")
        scanbutton.horizontal_padding_em = 1
        scanbutton.vertical_padding_em = 1
        scanbutton.set_on_clicked(self._on_scan_button)

        fileedit_layout = gui.Horiz()
        fileedit_layout.add_child(gui.Label("Model file"))
        fileedit_layout.add_child(self._fileedit)
        fileedit_layout.add_fixed(0.25 * em)
        fileedit_layout.add_child(filedlgbutton)
        # add to the top-level (vertical) layout
        gui_layout.add_child(fileedit_layout)

        w.add_child(gui_layout)
        w.add_child(self._widget3d)

    def _on_mouse_widget3d(self, event):
        #print(event.type)
        return gui.Widget.EventCallbackResult.IGNORED

    def _on_filedlg_button(self):
      #  filedlg = gui.FileDialog(gui.FileDialog.OPEN, "Select file", self.window.theme)
      #  filedlg.add_filter(".obj .ply .stl", "Triangle mesh (.obj, .ply, .stl)")
      #  filedlg.add_filter("", "All files")
      #  filedlg.set_on_cancel(self._on_filedlg_cancel)
      #  filedlg.set_on_done(self._on_filedlg_done)
      #  self.window.show_dialog(filedlg)
        if self._widget3d.scene.has_geometry('cloud2') :
            self._widget3d.scene.remove_geometry('cloud2')
        self._widget3d.scene.add_geometry('cloud2', self.point_cloud, self.material)

        #o3d.io.write_point_cloud("point_pipe.pcd", self.point_cloud)
        pcd2 = fp.get_flattened_pcds2(source=self.point_cloud,A=0,B=1,C=0,D=0,x0=0,y0=1000,z0=0)
        center, normal, radius, inliers = fp.get_cylinder(pcd2, thresh=0.1, maxIteration=1)
        mesh_cylinder = fp.get_clylinder_mesh(pcd2,center, normal, radius, inliers)
        if self._widget3d.scene.has_geometry('cylinder') :
            self._widget3d.scene.remove_geometry('cylinder')
        self._widget3d.scene.add_geometry('cylinder', mesh_cylinder, self.material)
        print("center: " + str(center))
        print("radius: " + str(radius))
        print("vecC: " + str(normal))
        print("inliers: ", inliers)
       # cil = pyrsc.Cylinder()
       # points = np.asarray(self.point_cloud.points)
       
       # center, normal, radius, inliers = cil.fit(points, thresh=0.02)
       # print("center: " + str(center))
       # print("radius: " + str(radius))
       # print("vecC: " + str(normal))
       # R = pyrsc.get_rotationMatrix_from_vectors([0, 0, 1], normal)

       # plane = self.point_cloud.select_by_index(inliers).paint_uniform_color([1, 0, 0])
      #  not_plane = self.point_cloud.select_by_index(inliers, invert=True)
      #  mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(origin=[0, 0, 0], size=0.2)
      #  cen = o3d.geometry.TriangleMesh.create_coordinate_frame(origin=center, size=0.5)
      #  mesh_rot = copy.deepcopy(mesh).rotate(R, center=[0, 0, 0])

      #  mesh_cylinder = o3d.geometry.TriangleMesh.create_cylinder(radius=radius, height=0.5)
      #  mesh_cylinder.compute_vertex_normals()
      #  mesh_cylinder.paint_uniform_color([0.1, 0.9, 0.1])
       # mesh_cylinder = mesh_cylinder.rotate(R, center=[0, 0, 0])
       # mesh_cylinder = mesh_cylinder.translate((center[0], center[1], center[2]))
 
       # self._widget3d.scene.add_geometry('mesh', mesh_cylinder, self.material)

    def _on_filedlg_cancel(self):
        self.window.close_dialog()

    def _on_filedlg_done(self, path):
        self._fileedit.text_value = path
        self.model_dir = os.path.normpath(path)
        # load model
        self.window.close_dialog()
    
    def _on_scan_button(self):
        self._widget3d.scene.add_geometry('cloud2', self.point_cloud, self.material)
    
    def callback(self, ros_cloud):
        self.point_cloud.points = o3d.utility.Vector3dVector(ros_numpy.point_cloud2
                                        .pointcloud2_to_xyz_array(ros_cloud))


def main():
    
    gui.Application.instance.initialize()
    w = WindowApp()
    gui.Application.instance.run()

if __name__ == "__main__":
    main()