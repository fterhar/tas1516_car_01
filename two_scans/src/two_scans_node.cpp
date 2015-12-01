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
        laser_geometry::LaserProjection projector_;
        tf::TransformListener tfListener_;

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
	
    front_scan_sub_ = node_.subscribe<sensor_msgs::LaserScan> ("/scan", 100, &My_Scan::frontScanCallback, this);
    back_scan_sub_ = node_.subscribe<sensor_msgs::LaserScan> ("/scan_back", 100, &My_Scan::backScanCallback, this);
    point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2> ("/two_scans", 100, false);
}

void My_Scan::frontScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    projector_.transformLaserScanToPointCloud("base_link", *scan, front_scan_cloud, tfListener_);
    received_front_scan = true;
    
    sendScan();
}

void My_Scan::backScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    projector_.transformLaserScanToPointCloud("base_link", *scan, back_scan_cloud, tfListener_);
    received_back_scan = true;
    
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
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "two_scan_publisher");

    My_Scan myScan;

    ros::spin();

    return 0;
}
