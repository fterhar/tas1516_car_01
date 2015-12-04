#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <string>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/**
 * Attention!!!
 * This node works only in a single thread. Otherwise mutual exclusion have to be added to provide parallelism.
 * Source: http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning
 */

class My_Scan {
     public:
        My_Scan();
        void frontScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
        void backScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
     private:
        ros::NodeHandle node_;
        laser_geometry::LaserProjection projector_front_;
        laser_geometry::LaserProjection projector_back_;
        tf::TransformListener tfListener_front_;
        tf::TransformListener tfListener_back_;

        ros::Publisher point_cloud_publisher_;
        ros::Subscriber front_scan_sub_;
        ros::Subscriber back_scan_sub_;
        
        bool front_init_done;
        bool back_init_done;
        
        /**
         * Parameters of the scanners
         */
        std::string front_frame_id;
        float front_angle_min;
        float front_angle_max;
        float front_angle_increment;
        float front_range_min;
        float front_range_max;
        std::string back_frame_id;
        float back_angle_min;
        float back_angle_max;
        float back_angle_increment;
        float back_range_min;
        float back_range_max;
        
        bool received_front_scan;
        sensor_msgs::PointCloud2 front_scan_cloud;
        bool received_back_scan;
        sensor_msgs::PointCloud2 back_scan_cloud;
        
        void sendScan();
};

My_Scan::My_Scan(){
	front_init_done = false;
	back_init_done = false;
	
	received_front_scan = false;
	received_back_scan = false;
	
    front_scan_sub_ = node_.subscribe<sensor_msgs::LaserScan> ("/scan", 10000, &My_Scan::frontScanCallback, this);
    back_scan_sub_ = node_.subscribe<sensor_msgs::LaserScan> ("/scan_back", 10000, &My_Scan::backScanCallback, this);
    point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2> ("/two_scans_cloud", 10000, false);
    
    tfListener_front_.setExtrapolationLimit(ros::Duration(0.1));
    tfListener_back_.setExtrapolationLimit(ros::Duration(0.1));
}

void My_Scan::frontScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    if(!tfListener_front_.waitForTransform(scan->header.frame_id, "/laser", scan->header.stamp + ros::Duration().fromSec(scan->ranges.size()*0.5),ros::Duration(2.0))){
     	return;
    }
	
	try {
        projector_front_.transformLaserScanToPointCloud("/laser", *scan, front_scan_cloud, tfListener_front_);
    } catch (int err) {ROS_INFO("ERROR front");}
    received_front_scan = true;
    
    ROS_INFO("Received front scan");
    sendScan();
}

void My_Scan::backScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    if(!tfListener_back_.waitForTransform(scan->header.frame_id, "/laser", scan->header.stamp + ros::Duration().fromSec(scan->ranges.size()*0.5),ros::Duration(2.0))){
     	return;
    }
	
	try {
        projector_back_.transformLaserScanToPointCloud("/laser", *scan, back_scan_cloud, tfListener_back_);
    } catch (int err) {ROS_INFO("ERROR back");}
    received_back_scan = true;
    
    ROS_INFO("Received back scan");
    sendScan();
}

void My_Scan::sendScan(){
    // combine clouds
    sensor_msgs::PointCloud2 combined_scan;
    pcl::concatenatePointCloud(front_scan_cloud, back_scan_cloud, combined_scan);
    
    // reset
    received_front_scan = false;
    received_back_scan = false;
    
    point_cloud_publisher_.publish(combined_scan);
    ROS_INFO("Published to two_scans_cloud");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "two_scan_publisher");

    My_Scan myScan;

    ros::spin();

    return 0;
}
