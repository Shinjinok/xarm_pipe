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
from moveit_commander.conversions import pose_to_list
import os
import sys
import numpy as np
#import test3dplot
import rospy
sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))

## First we start with the standard ros Python import line:
import roslib
roslib.load_manifest('rviz_python_tutorial')

import pipe_scan_function as psf

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
## The MyViz class is the main container widget.
from PyQt5.QtCore import QObject


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
        reader.readFile( config, "myviz.rviz")#"config.myviz" )
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
        #self.frame.setMenuBar( None )
        #self.frame.setStatusBar( None )
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
        
        
        
        """thickness_slider = QSlider( Qt.Horizontal )
        thickness_slider.setTracking( True )
        thickness_slider.setMinimum( 1 )
        thickness_slider.setMaximum( 180 )
        thickness_slider.setValue(90)
        thickness_slider.valueChanged.connect( self.onThicknessSliderChanged )
        layout.addWidget( thickness_slider )"""
        
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

        go_button = QPushButton( "Go Target" )
        go_button.clicked.connect( self.onGoButtonClick )
        h_layout.addWidget( go_button )

        save_button = QPushButton( "Save PCD" )
        save_button.clicked.connect( self.onSaveButtonClick )
        h_layout.addWidget( save_button )

        layout.addLayout( h_layout )

        h_layout2 = QHBoxLayout()
        

        pitch_label = QLabel("Pitch")
        h_layout2.addWidget(pitch_label)
        

        self.pitch_text_box = QTextEdit("0.0")
        self.pitch_text_box.setMaximumHeight(pitch_label.sizeHint().height()*2)
        h_layout2.addWidget(self.pitch_text_box)
        
        L_dist_label = QLabel("L_dist(m)")
        h_layout2.addWidget(L_dist_label)

        self.L_dist_text_box = QTextEdit("0.2")
        self.L_dist_text_box.setMaximumHeight(L_dist_label.sizeHint().height()*2)
        h_layout2.addWidget(self.L_dist_text_box)

        

        R_dist_label = QLabel("R_dist(m)")
        h_layout2.addWidget(R_dist_label)

        self.R_dist_text_box = QTextEdit("0.2")

        self.R_dist_text_box.setMaximumHeight(R_dist_label.sizeHint().height()*2)
        h_layout2.addWidget(self.R_dist_text_box)

        go_rotation_button = QPushButton( "Go Rotation" )
        go_rotation_button.clicked.connect( self.onGoRotationClick )
        h_layout2.addWidget( go_rotation_button )

        layout.addLayout( h_layout2 )       
        
        
        self.setLayout( layout )
        
        dual = True
        self.psf = psf.PipeScanFunction(dual)
        
    
  

    ## Handle GUI events
    ## ^^^^^^^^^^^^^^^^^
    ##
    ## After the constructor, for this example the class just needs to
    ## respond to GUI events.  Here is the slider callback.
    ## rviz.Display is a subclass of rviz.Property.  Each Property can
    ## have sub-properties, forming a tree.  To change a Property of a
    ## Display, use the subProp() function to walk down the tree to
    ## find the child you need.
    """def onThicknessSliderChanged( self, new_value ):
        self.psf.change_angle_thread(new_value)"""

    ## The view buttons just call switchToView() with the name of a saved view.
    def onHomeButtonClick( self ):
        self.psf.command_thread("home")
        #self.text_box.setText("home")   

    def onScanHButtonClick( self ):
        self.psf.command_thread("hscan")
        #self.text_box.setText("hscan")   

    def onScanVButtonClick( self ):
        self.psf.command_thread("vscan")
        #self.text_box.setText("hscan")  

    def onSaveButtonClick( self ):
        self.psf.command_thread("savePCD")
        #self.text_box.setText("savePCD")  

    def onGoButtonClick( self ):
        ldist = float(self.L_dist_text_box.toPlainText())
        rdist = float(self.R_dist_text_box.toPlainText())
        self.psf.go_target_thread(ldist,rdist)
        #self.text_box.setText("go_target")      
    def onGoRotationClick( self ):
       # roll = float(self.roll_text_box.toPlainText())/180.*3.14159
        pitch = float(self.pitch_text_box.toPlainText())/180.*3.14159
       # yaw = float(self.yaw_text_box.toPlainText())/180.*3.14159
       # new_value = np.array([roll,pitch,yaw])
        ldist = float(self.L_dist_text_box.toPlainText())
        rdist = float(self.R_dist_text_box.toPlainText())
        print("set rotation :",pitch)
        self.psf.change_angle_thread(pitch,ldist,rdist)

         
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
    myviz.resize( 1000, 500 )
    myviz.show()
    app.exec_()