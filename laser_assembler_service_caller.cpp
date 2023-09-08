#include <ros/ros.h>
#include <laser_geometry/laser_geometry.h>
#include <laser_assembler/AssembleScans.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>


#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>




int main(int argc, char **argv)
{
    ros::init(argc, argv, "assemble_scans_service_caller");
    ros::NodeHandle n;
    ros::service::waitForService("assemble_scans");
    ros::ServiceClient client = n.serviceClient<laser_assembler::AssembleScans>("assemble_scans");
    laser_assembler::AssembleScans srv;

    //ros::Publisher point_cloud_publisher_;
    ros::Publisher point_cloud2_publisher_;
    //point_cloud_publisher_ = n.advertise<sensor_msgs::PointCloud> ("/cloud", 100, false);
                                        //const std::string& topic 퍼블리쉬 할 토픽 이름 
                                        //uint32_t queue_size 퍼블리쉬 메시지 대기열의 크기, 오래된 메세지 부터 삭제,
                                        // bool latch = false 대기열에서 마지막 메세지 데이터를 전송 여부
    point_cloud2_publisher_ = n.advertise<sensor_msgs::PointCloud2> ("/cloud2", 100, false);


    ros::Rate loop_rate(5 /*10*/);//Hz
    sensor_msgs::PointCloud2 point_cloud2;
    while (ros::ok())
    {
        srv.request.begin = ros::Time(0,0);
        srv.request.end   = ros::Time::now();
        if (client.call(srv))
        {
            printf("Got cloud with %lu points..\n", srv.response.cloud.points.size());
            if(srv.response.cloud.points.size()>0)
            {
                //point_cloud_publisher_.publish(srv.response.cloud);
                sensor_msgs::convertPointCloudToPointCloud2(srv.response.cloud, point_cloud2);
                point_cloud2_publisher_.publish(point_cloud2);
            }
        }
        else
        printf("Service call failed\n");
        loop_rate.sleep();
    }
}
