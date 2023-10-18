
import moveit_function as mf
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped
import numpy as np #pip install numpy==1.20.3
import open3d as o3d
import threading
import flattten_pcd as fp
import ros_numpy
import rospy
import rviz_tools
import find_cylinder as fc



class PipeScanFunction():

    def __init__(self, dual):
        self.pcd2 = o3d.geometry.PointCloud()
        self.pcd3 = o3d.geometry.PointCloud()
        self.Lmargin = 0.05
        self.Rmargin = 0.05
        self.tutorial = mf.MoveGroupPythonInterfaceTutorial(dual)
        self.markers = rviz_tools.RvizMarkers('world', 'visualization_marker')
        self.pos_goal = np.array([0.3,0,0.3,0,0,0])
        self.target_center = np.array([0.3, 0, 0.5])
        self.target_radius = 0
        self.center = []
        self.normal = []
        self.radius = []
        self.deg = 0
        rospy.Subscriber('/cloud2', PointCloud2, self.callback_point_cloud2)
        rospy.Subscriber('/clicked_point', PointStamped, self.callback_clicked_point)

        self.fname = "point_pipe.pcd"
        self.fc = fc.FindCylinder()
        self.dual = dual
        print("Dual mode: ", dual)

    def command_thread(self, msg):
        if msg == "home":
            t = threading.Thread(target = self.home)
            t.daemon = True
            t.start() 
        if msg == "hscan":
            t = threading.Thread(target=self.hscan)
            t.daemon = True
            t.start()
        if msg == "vscan":
            t = threading.Thread(target=self.vscan)
            t.daemon = True
            t.start()
        if msg == "savePCD":
            t = threading.Thread(target=self.savePCD)
            t.daemon = True
            t.start()

    def go_target_thread(self,ldist, rdist):
        t = threading.Thread(target=self.go_target,args=(ldist, rdist))
        t.daemon = True
        t.start()    
    
    def change_angle_thread(self, rpy, ldist, rdist):
        
        t = threading.Thread(target = self.change_deg, args=(rpy, ldist, rdist))
        t.daemon = True
        t.start()

    def change_deg(self, rpy, ldist, rdist):
        move_group = "L_xarm6"
   
        print( "Tcp Go to Cylinder Center Position")
        print("R ", self.target_radius," pitch ",rpy," ldist ", ldist ,"\n")

        self.pos_goal[0] = self.target_center[0] - (self.target_radius+ldist+self.Lmargin)*np.cos(rpy)
        self.pos_goal[1] = self.target_center[1]
        self.pos_goal[2] = self.target_center[2] + (self.target_radius+ldist+self.Lmargin)*np.sin(rpy)
        self.pos_goal[3] = 0
        self.pos_goal[4] = rpy
        self.pos_goal[5] = 0
        

        print(self.pos_goal)
        self.tutorial.go_to_pose_goal(move_group, self.pos_goal,0.5)
        
        if self.dual :
            print( "Pad Go to Cylinder Center Position")
            self.pos_goal[0] = self.target_center[0] + (self.target_radius + rdist +self.Rmargin)*np.cos(rpy)
            self.pos_goal[1] = self.target_center[1] 
            self.pos_goal[2] = self.target_center[2] - (self.target_radius + rdist +self.Rmargin)*np.sin(rpy)
            self.pos_goal[3] = 0
            self.pos_goal[4] = rpy
            self.pos_goal[5] = 0
            move_group = "R_xarm6"
            print(self.pos_goal)
            self.tutorial.go_to_pose_goal(move_group, self.pos_goal,0.5)
            
    def home(self):
        print("Left arm move home")
        
        move_group = "L_xarm6"
        joint = np.array([0,0,0,0,-3.14/2,0])
        speed_factor = 0.5
        self.tutorial.go_to_joint_state(move_group,joint,speed_factor)
        if self.dual == True :
            print("Right arm move home")
            move_group = "R_xarm6"
            joint = np.array([0,-1,0,0,-3.14/2,0])
            self.tutorial.go_to_joint_state(move_group,joint,speed_factor)

    def hscan(self):

        self.markers.deleteAllMarkers()

        print( " Go to start position")
        move_group = "L_xarm6"
        position = np.array([0.3, 0.3,0.3])
        speed_factor = 0.1
        plan, fraction = self.tutorial.plan_cartesian_path(move_group,position,speed_factor)
        self.tutorial.execute_plan(plan,move_group)
        print( "Horizental Scaning")
        position = np.array([0.3, -0.3,0.3])
        speed_factor = 0.001
        plan, fraction = self.tutorial.plan_cartesian_path(move_group,position,speed_factor)
        self.tutorial.execute_plan(plan,move_group)
        print( "Done")


        self.pcd3.points = self.pcd2.points
        color = [0,1,0,0.5]
        self.center, self.normal, self.radius = self.find_cylinder(self.pcd3)
        
        if len(self.center) == 1 :
            get = np.array(self.center[0])
            self.target_center[0] = get[0]
            self.target_center[1] = get[1]
            self.target_center[2] = get[2]
            self.target_radius = self.radius[0]
            
            self.tutorial.add_scene_pipe(orientation=self.normal[0],position=self.center[0],radius=self.radius[0],height= 0.5)


        for c in range(len(self.center)):
            print("Cylinder Center pos: ", self.center[c] ," radius", self.radius[c])
            P=self.fc.get_clylinder_pos(self.center[c],self.normal[c])

            self.markers.publishCylinder(pose = P, color = color ,height= 0.2, radius = self.radius[c]*2, lifetime=0) 

            
            # pose, color, height, radius, lifetime  
        print("Marker published") 


    def find_cylinder(self, pcd):
        print("Finding Cylinder ...")
        center, normal, radius = fp.get_cylinder(pcd, thresh=0.01, maxIteration=10000)
                #pcd2 = fp.get_flattened_pcds2(source=point_cloud,A=0,B=1,C=0,D=0,x0=0,y0=1000,z0=0)
        print("Get Cylinder Center pos number: ", len(center))
        
        move_group = "L_xarm6"
        joint = np.array([0,0,0,0,-3.14/2,0])
        speed_factor = 0.5
        self.tutorial.go_to_joint_state(move_group,joint,speed_factor)

        return center, normal, radius  
 
    def vscan(self):

        print( " Go to start position")
        self.tutorial.go_to_joint_state(0,0,0,0,-3.14/2,3.14/2,1)
        plan, fraction = self.tutorial.plan_cartesian_path(np.array([0.3, 0, 0.5]),0.1)
        self.tutorial.execute_plan(plan)
        print( "Vertical Scaning")
        plan, fraction = self.tutorial.plan_cartesian_path(np.array([0.3, 0, 0.8]),0.001)
        self.tutorial.execute_plan(plan)
        print( "Done")

    def go_target(self,ldist,rdist):
        move_group = "L_xarm6"
                
        print( "Tcp Go to Cylinder Center Position")
        self.pos_goal[0] = self.target_center[0] - (self.target_radius + ldist +self.Lmargin)
        self.pos_goal[1] = self.target_center[1]
        self.pos_goal[2] = self.target_center[2]
        self.pos_goal[3] = 0
        self.pos_goal[4] = 0
        self.pos_goal[5] = 0
       

        print(self.pos_goal)
        self.tutorial.go_to_pose_goal(move_group, self.pos_goal,0.5)

        if self.dual :
            move_group = "R_xarm6"
            print( "Pad Go to Cylinder Center Position")
            self.pos_goal[0] = self.target_center[0] + (self.target_radius + rdist +self.Rmargin)
            self.pos_goal[1] = self.target_center[1]
            self.pos_goal[2] = self.target_center[2]
            self.pos_goal[3] = 0
            self.pos_goal[4] = 0
            self.pos_goal[5] = 0
            
            print(self.pos_goal)
            self.tutorial.go_to_pose_goal(move_group, self.pos_goal,0.5)


    def savePCD(self):
        
        #filename, pointcloud, write_ascii=False, compressed=False, print_progress=False)
        o3d.io.write_point_cloud(filename=self.fname, pointcloud=self.pcd3)
        print(self.fname," saved")


    def callback_point_cloud2(self, ros_cloud):
        
        self.pcd2.points = o3d.utility.Vector3dVector(ros_numpy.point_cloud2
                                        .pointcloud2_to_xyz_array(ros_cloud))
        
    def callback_clicked_point(self, msg):
        point = PointStamped()
        point.header.stamp = rospy.Time.now()
        point.header.frame_id = "/map"
        point.point.x = msg.point.x      
        point.point.y = msg.point.y
        point.point.z = msg.point.z
        print("coordinates:x=%f y=%f z=%f" %(point.point.x, point.point.y,point.point.z))
        print("len(center)",len(self.center))
        for c in range(len(self.center)):
            get = np.array(self.center[c])  
            print("center x ", get[0])
            if np.abs(get[0] -  point.point.x) < 0.2 :
                print("Cylinder Center pos: ", self.center[c] ," radius", self.radius[c])
                P=self.fc.get_clylinder_pos(self.center[c],self.normal[c])
                self.markers.deleteAllMarkers()
                color = [0,1,0,0.5]
                self.markers.publishCylinder(pose = P, color = color ,height= 0.2, radius = self.radius[c]*2, lifetime=0) 
                print("Marker published!!")
                self.target_center[0] = get[0]
                self.target_center[1] = get[1]
                self.target_center[2] = get[2]
                self.target_radius = self.radius[c]*2
                
                self.tutorial.add_scene_pipe(orientation=self.normal[c],position=self.center[c],radius=self.radius[c],height= 0.5)
                
            # pose, color, height, radius, lifetime  
        

    