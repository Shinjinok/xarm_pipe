#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan

import numpy as np
import pandas as pd
   
def callback(msg):
    data = np.array(msg.ranges)
    df = pd.DataFrame(data, columns = ['ranges'])
    count = df.groupby(np.isinf(df['ranges'])).count()
    #print (len(msg.ranges))
    print (count)
  
def listener():

    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/scan', LaserScan, callback)  
    rospy.spin()
if __name__ == '__main__':
    listener()