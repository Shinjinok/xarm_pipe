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
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import os
import sys
import time
import math
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3, Polygon
from tf import transformations # rotation_matrix(), concatenate_matrices()

import moveit_function

import rospy
sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))

from xarm.wrapper import XArmAPI
import rviz_tools
## First we start with the standard ros Python import line:
import roslib
roslib.load_manifest('rviz_python_tutorial')

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


tutorial = moveit_function.MoveGroupPythonIntefaceTutorial()
##rospy.init_node('marker', anonymous=True, log_level=rospy.INFO, disable_signals=False)
## The MyViz class is the main container widget.
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
        
        top_button = QPushButton( "Start Scan" )
        top_button.clicked.connect( self.onTopButtonClick )
        h_layout.addWidget( top_button )
        
        side_button = QPushButton( "Stop Scan" )
        side_button.clicked.connect( self.onSideButtonClick )
        h_layout.addWidget( side_button )
        
        marker_button = QPushButton( "marker" )
        marker_button.clicked.connect( self.onMarkerButtonClick )
        h_layout.addWidget( marker_button )
        
        layout.addLayout( h_layout )
        
        self.setLayout( layout )

        


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
    def onTopButtonClick( self ):
        print( " tutorial.go_to_joint_state1")
        tutorial.go_to_joint_state(0.0,0.0,0.0,0.0,-3.14/2.0,-3.14/2.0,1)
        
    def onSideButtonClick( self ):
        print( " tutorial.go_to_joint_state2")
        tutorial.go_to_joint_state(0.0,0.0,0.0,0.0,-3.14/2.0,3.14/2.0,1)

    def onMarkerButtonClick( self ):
        # Cube / Cuboid:
        # Publish a cuboid using a numpy transform matrix
        markers = rviz_tools.RvizMarkers ('world', 'visualization_marker')
        T = transformations.translation_matrix((0.6,2.2,0))
        scale = Vector3(1.5,5.2,0.2)
        markers.publishCube(T, 'red', scale, 5.0) # pose, color, scale, lifetime
            
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
    myviz.resize( 1000, 1000 )
    myviz.show()

    
       
    app.exec_()