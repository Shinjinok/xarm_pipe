#!/usr/bin/env python

## BEGIN_TUTORIAL
##
## Imports
## ^^^^^^^
##
"""
Description: Move Joint
"""
import sys
import rospy

from math import pi

from moveit_commander.conversions import pose_to_list
import os
import sys


from tf import transformations # rotation_matrix(), concatenate_matrices()

import moveit_function2
#import test3dplot
import rospy
sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))

from xarm.wrapper import XArmAPI
import rviz_tools
## First we start with the standard ros Python import line:
import roslib
roslib.load_manifest('rviz_python_tutorial')

import flattten_pcd as fp
import open3d as o3d
import ros_numpy #sudo apt-get install ros-noetic-ros-numpy
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped
import numpy as np #pip install numpy==1.20.3

## Next import all the Qt bindings into the current namespace, for
## convenience.  This uses the "python_qt_binding" package which hides
## differences between PyQt and PySide, and works if at least one of
## the two is installed.  The RViz Python bindings use
## python_qt_binding internally, so you should use it here as well.
from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
try:
    from python_qt_binding.QtWidgets import *
except ImportError:
    pass

## Finally import the RViz bindings themselves.
from rviz import bindings as rviz
from threading import Thread
from multiprocessing import Process, Queue

##rospy.init_node('marker', anonymous=True, log_level=rospy.INFO, disable_signals=False)
## The MyViz class is the main container widget.
from PyQt5.QtCore import QObject

class Worker(QThread):

    def __init__(self):
        super().__init__()
        self.markers = rviz_tools.RvizMarkers('world', 'visualization_marker')
        self.tutorial = moveit_function2.MoveGroupPythonInterfaceTutorial()
        self.working = ""
        self.pos_goal = np.array([0.3,0,0.3,0,0,0])
        self.pcd = o3d.geometry.PointCloud()
        self.center = np.array([0.3, 0, 0.5])

    def run(self):
       
        if self.working == "home":
            move_group = "L_xarm6"
            joint = np.array([0,0,0,0,-3.14/2,0])
            speed_factor = 1
            self.tutorial.go_to_joint_state(move_group,joint,speed_factor)
            move_group = "R_xarm6"
            joint = np.array([0,-1,0,0,-3.14/2,0])
            self.tutorial.go_to_joint_state(move_group,joint,speed_factor)

        if self.working == "hscan":
            print( " Go to start position")
            move_group = "L_xarm6"
            position = np.array([0.3, 0.3,0.3])
            speed_factor = 0.1
            plan, fraction = self.tutorial.plan_cartesian_path(move_group,position,speed_factor)
            self.tutorial.execute_plan(plan,move_group)
            print( "Horizental Scaning")
            position = np.array([0.3, -0.3,0.3])
            speed_factor = 0.01
            plan, fraction = self.tutorial.plan_cartesian_path(move_group,position,speed_factor)
            self.tutorial.execute_plan(plan,move_group)
            print( "Done")

        if self.working == "vscan":
            print( " Go to start position")
            self.tutorial.go_to_joint_state(0,0,0,0,-3.14/2,3.14/2,1)
            plan, fraction = self.tutorial.plan_cartesian_path(np.array([0.3, 0, 0.5]),0.1)
            self.tutorial.execute_plan(plan)
            print( "Vertical Scaning")
            plan, fraction = self.tutorial.plan_cartesian_path(np.array([0.3, 0, 0.8]),0.001)
            self.tutorial.execute_plan(plan)
            print( "Done")

        if self.working == "go_target":
            print( "Tcp Go to Cylinder Center Position")
            self.pos_goal[0] = self.center[0] - 0.2
            self.pos_goal[1] = self.center[1]
            self.pos_goal[2] = self.center[2]
            self.pos_goal[3] = 0
            self.pos_goal[4] = 0
            self.pos_goal[5] = 0
            move_group = "L_xarm6"
            print(self.center)
            print(self.pos_goal)
            self.tutorial.go_to_pose_goal(move_group, self.pos_goal,0.5)

            print( "Pad Go to Cylinder Center Position")
            self.pos_goal[0] = self.center[0] + 0.2
            self.pos_goal[1] = self.center[1]
            self.pos_goal[2] = self.center[2]
            self.pos_goal[3] = 0
            self.pos_goal[4] = 0
            self.pos_goal[5] = 0
            move_group = "R_xarm6"
            print(self.pos_goal)
            self.tutorial.go_to_pose_goal(move_group, self.pos_goal,0.5)


        if self.working == "marker":
            #pcd2 = fp.get_flattened_pcds2(source=self.point_cloud,A=0,B=1,C=0,D=0,x0=0,y0=1000,z0=0)
            print("Finding Cylinder ...")
            center, normal, radius = fp.get_cylinder(self.pcd, thresh=0.01, maxIteration=10000)
            #self.pos_goal[0] = center[0]
            #self.pos_goal[2] = center[2]
            #self.targepoint = center
            print("Cylinder Center pos: ", center)
            for c in range(len(center)):
                print("Cylinder Center pos: ", center[c] ," radius", radius[c])
                P=fp.get_clylinder_pos(center[c],normal[c])
                #publishCylinder(self, pose, color, height, radius, lifetime=None):
                self.markers.publishCylinder(pose = P, color = [0,1,0,0.5] ,height= 0.2,
                    radius = radius[c]*2, lifetime=0) # pose, color, height, radius, lifetime    
                self.center = center[c]
            print("Marker published")
            
        


class MyViz( QWidget ):

    ## MyViz Constructor
    ## ^^^^^^^^^^^^^^^^^
    ##
    ## Its constructor creates and configures all the component widgets:
    ## frame, thickness_slider, top_button, and side_button, and adds them
    ## to layouts.
    def __init__(self):
        QWidget.__init__(self)

        ## rviz.VisualizationFrame is the main container widget of the
        ## regular RViz application, with menus, a toolbar, a status
        ## bar, and many docked subpanels.  In this example, we
        ## disable everything so that the only thing visible is the 3D
        ## render window.
        self.frame = rviz.VisualizationFrame()

        ## The "splash path" is the full path of an image file which
        ## gets shown during loading.  Setting it to the empty string
        ## suppresses that behavior.
        self.frame.setSplashPath( "" )

        ## VisualizationFrame.initialize() must be called before
        ## VisualizationFrame.load().  In fact it must be called
        ## before most interactions with RViz classes because it
        ## instantiates and initializes the VisualizationManager,
        ## which is the central class of RViz.
        self.frame.initialize()

        ## The reader reads config file data into the config object.
        ## VisualizationFrame reads its data from the config object.
        reader = rviz.YamlConfigReader()
        config = rviz.Config()
        reader.readFile( config, "config.myviz" )
        self.frame.load( config )

        ## You can also store any other application data you like in
        ## the config object.  Here we read the window title from the
        ## map key called "Title", which has been added by hand to the
        ## config file.
        self.setWindowTitle( config.mapGetChild( "Title" ).getValue() )

        ## Here we disable the menu bar (from the top), status bar
        ## (from the bottom), and the "hide-docks" buttons, which are
        ## the tall skinny buttons on the left and right sides of the
        ## main render window.
        self.frame.setMenuBar( None )
        self.frame.setStatusBar( None )
        self.frame.setHideButtonVisibility( False )

        ## frame.getManager() returns the VisualizationManager
        ## instance, which is a very central class.  It has pointers
        ## to other manager objects and is generally required to make
        ## any changes in an rviz instance.
        self.manager = self.frame.getManager()

        ## Since the config file is part of the source code for this
        ## example, we know that the first display in the list is the
        ## grid we want to control.  Here we just save a reference to
        ## it for later.
        self.grid_display = self.manager.getRootDisplayGroup().getDisplayAt( 0 )
        
        ## Here we create the layout and other widgets in the usual Qt way.
        layout = QVBoxLayout()
        layout.addWidget( self.frame )
        
        thickness_slider = QSlider( Qt.Horizontal )
        thickness_slider.setTracking( True )
        thickness_slider.setMinimum( 1 )
        thickness_slider.setMaximum( 1000 )
        thickness_slider.valueChanged.connect( self.onThicknessSliderChanged )
        layout.addWidget( thickness_slider )
        
        h_layout = QHBoxLayout()
        
        home_button = QPushButton( "Home" )
        home_button.clicked.connect( self.onHomeButtonClick )
        h_layout.addWidget( home_button )
        
        scanH_button = QPushButton( "H Scan" )
        scanH_button.clicked.connect( self.onScanHButtonClick )
        h_layout.addWidget( scanH_button )

        scanV_button = QPushButton( "V Scan" )
        scanV_button.clicked.connect( self.onScanVButtonClick )
        h_layout.addWidget( scanV_button )

        save_button = QPushButton( "Save PCD" )
        save_button.clicked.connect( self.onSaveButtonClick )
        h_layout.addWidget( save_button )

        go_button = QPushButton( "Go Target" )
        go_button.clicked.connect( self.onGoButtonClick )
        h_layout.addWidget( go_button )
        
        marker_button = QPushButton( "Marker" )
        marker_button.clicked.connect( self.onMarkerButtonClick )
        h_layout.addWidget( marker_button )

        
        
        layout.addLayout( h_layout )
        
        self.setLayout( layout )

        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)
        rospy.Subscriber('/cloud2', PointCloud2, self.callback_point_cloud2)
        rospy.point_pub = rospy.Subscriber('/clicked_point', PointStamped, self.callback_clicked_point)
        self.point_cloud = o3d.geometry.PointCloud()
        

        self.worker = Worker()
    
  

    ## Handle GUI events
    ## ^^^^^^^^^^^^^^^^^
    ##
    ## After the constructor, for this example the class just needs to
    ## respond to GUI events.  Here is the slider callback.
    ## rviz.Display is a subclass of rviz.Property.  Each Property can
    ## have sub-properties, forming a tree.  To change a Property of a
    ## Display, use the subProp() function to walk down the tree to
    ## find the child you need.
    def onThicknessSliderChanged( self, new_value ):
        if self.grid_display != None:
            self.grid_display.subProp( "Line Style" ).subProp( "Line Width" ).setValue( new_value / 1000.0 )

    ## The view buttons just call switchToView() with the name of a saved view.
    def onHomeButtonClick( self ):
        self.worker.start()
        self.worker.working = "home"
        
    def onScanHButtonClick( self ):
        self.worker.start()
        self.worker.working = "hscan"

    def onScanVButtonClick( self ):
        self.worker.start()
        self.worker.working = "vscan"

    def onSaveButtonClick( self ):
        o3d.io.write_point_cloud("point_pipe.pcd", self.point_cloud)

    def onGoButtonClick( self ):
        self.worker.start()
        self.worker.working = "go_target" 
        #self.worker.pos_goal = np.array([0, 0, 0, 0, 0, 0])  #x,y,z,roll,pitch,yaw 
                

    def onMarkerButtonClick( self ):
        self.worker.start()
        self.worker.working = "marker"
        self.worker.pcd = self.point_cloud
        
           
    ## switchToView() works by looping over the views saved in the
    ## ViewManager and looking for one with a matching name.
    ##
    ## view_man.setCurrentFrom() takes the saved view
    ## instance and copies it to set the current view
    ## controller.
    def switchToView( self, view_name ):
        view_man = self.manager.getViewManager()
        for i in range( view_man.getNumViews() ):
            if view_man.getViewAt( i ).getName() == view_name:
                view_man.setCurrentFrom( view_man.getViewAt( i ))
                return
        print( "Did not find view named %s." % view_name )

    def callback_point_cloud2(self, ros_cloud):
        self.point_cloud.points = o3d.utility.Vector3dVector(ros_numpy.point_cloud2
                                        .pointcloud2_to_xyz_array(ros_cloud))
    def callback_clicked_point(self, msg):
        point = PointStamped()
        point.header.stamp = rospy.Time.now()
        point.header.frame_id = "/map"
        point.point.x = msg.point.x      
        point.point.y = msg.point.y
        point.point.z = msg.point.z
        print("coordinates:x=%f y=%f" %(point.point.x, point.point.y))
 
## Start the Application
## ^^^^^^^^^^^^^^^^^^^^^
##
## That is just about it.  All that's left is the standard Qt
## top-level application code: create a QApplication, instantiate our
## class, and start Qt's main event loop (app.exec_()).

#######################################################
"""
Just for test example
"""
if len(sys.argv) >= 2:
    ip = sys.argv[1]
else:
    try:
        from configparser import ConfigParser
        parser = ConfigParser()
        parser.read('./robot.conf')
        ip = parser.get('xArm', 'ip')
    except:
        ip = input('Please input the xArm ip address:')
        if not ip:
            print('input error, exit')
            sys.exit(1)
            
        
            
if __name__ == '__main__':
    app = QApplication( sys.argv )
    
    myviz = MyViz()
    myviz.resize( 1000, 500 )
    myviz.show()
    app.exec_()