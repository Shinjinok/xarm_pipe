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


if __name__ == "__main__":
    
    
    rospy.Subscriber('/cloud2', PointCloud2, callback)  
    
    try: 
        tutorial = moveit_function2.MoveGroupPythonInterfaceTutorial()
        i=0
        while i < 10000:
            # tutorial.go_to_joint_state1()
            print( " tutorial.go_to_joint_state1")
            ret=tutorial.go_to_joint_state(0.0,0.0,0.0,0.0,-3.14/2.0,-3.14/4.0,1)
            print(ret)
            print( " tutorial.go_to_joint_state2")
            ret = tutorial.go_to_joint_state(0.0,0.0,0.0,0.0,-3.14/2.0,3.14/4.0,1)
            print(ret)
            i = i + 1
            print(i)
            

            

    except rospy.ROSInterruptException as e:
        print(e)

   

   