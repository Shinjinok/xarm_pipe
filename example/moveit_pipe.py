import moveit_function2
import time
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import pcl #sudo apt install python3-pcl




def callback(ros_cloud):

    points_list = []

    for data in pc2.read_points(ros_cloud, skip_nans=True):
        points_list.append([data[0], data[1], data[2], data[3]])
   # print(len(points_list))
import numpy as np # Scientific computing library for Python
 
def get_quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]

if __name__ == "__main__":
    
    
    rospy.Subscriber('/cloud2', PointCloud2, callback)

    
    
    try: 
        tutorial = moveit_function2.MoveGroupPythonInterfaceTutorial()
        print( " Go to start position")
    
        tutorial.go_to_joint_state(0,0,0,0,-3.14/2,0,1)
        i=0
        qt = get_quaternion_from_euler(-1.507,0,0)
        while i < 10000:
            # tutorial.go_to_joint_state1()
            print( " tutorial.go_to_joint_state1")
            #ret=tutorial.go_to_joint_state(0.0,0.0,0.0,0.0,-3.14/2.0,-3.14/6.0,1)
            
            #ret=tutorial.go_to_pose_goal([0.2,0.4,0.2],qt)

            plan, fraction = tutorial.plan_cartesian_path([0.3,0.3,0.3],0.01)
            tutorial.execute_plan(plan)
            #print(ret)
            print( " tutorial.go_to_joint_state2")
            #ret = tutorial.go_to_joint_state(0.0,0.0,0.0,0.0,-3.14/2.0,3.14/6.0,1)
            #ret=tutorial.go_to_pose_goal([0.2,-0.4,0.2], qt)
            plan, fraction = tutorial.plan_cartesian_path([0.3,-0.3,0.3],0.01)
            tutorial.execute_plan(plan)
            
            i = i + 1
            print(i)
            

            

    except rospy.ROSInterruptException as e:
        print(e)

   

   