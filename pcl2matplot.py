import matplotlib.pyplot as plt
import rospy
import tf
from sensor_msgs.msg import PointCloud2
from tf.transformations import quaternion_matrix
import numpy as np
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import axes3d, Axes3D
import pcl_helper

class Visualiser:
    def __init__(self):
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(projection='3d')
        self.ln, = plt.plot([], [], [] ,'ro')
        self.x_data, self.y_data, self.z_data = [] , [] , []

    def plot_init(self):
        self.ax.set_xlim(-10000, 10000)
        self.ax.set_ylim(-10000, 10000)
        self.ax.set_zlim(-10000, 10000)
        return self.ln

    def odom_callback(self, msg):

        x_index = len(self.x_data)
        self.x_data.append(x_index+1)
        y_index = len(self.y_data)
        self.y_data.append(y_index+1)
        z_index = len(self.z_data)
        self.z_data.append(z_index+1)
    
    def update_plot(self, frame):
        self.ln.set_data(self.x_data, self.y_data, self.z_data)
        print(self.x_data)
        return self.ln


rospy.init_node('publisher_node')
vis = Visualiser()
sub = rospy.Subscriber('/cloud2', PointCloud2, vis.odom_callback)

ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init)
plt.show(block=True) 