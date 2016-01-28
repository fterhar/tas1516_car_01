#include <stdlib.h>
#include <time.h> 
#include <string>
#include <iostream>

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

/*
 * Constants
 */
 
 /*
  * If a scanner is already using the wished topic (e.g. "scan"), you should use an other topic name and change the config in the nodes where the combined scanner is required.
  * E.g.
  * AMCL is listening on the topic "scan" (default behaviour). The topic could be remapped with the following line:
  * <!-- Run AMCL -->
  * <node pkg="amcl" type="amcl" name="amcl" respawn="true">
  *     ...
  *     <remap from="scan" to="/two_scan"/>
  *     ...
  * </node>
  */
const std::string COMBINED_LASER_TOPIC = "two_scans_cloud";

/*
 * The target frame, which will be used in the new published topic
 * Please check that the frame is defined: 
 * E.g.
 * <!--(Static) transformation for relation between laser frame and base_link frame -->
 * <node pkg="tf" type="static_transform_publisher" name="base_link_to_laser" args="0.28 0.0 0.18 0.0 0.0 0.0 /base_link /laser 40" />
 * 
 * It is also important that the back scanner has a well defined frame for transformation:
 * 
 * E.g.
 * <!-- Added: the back scanner requires a own coordiante system -->
 * <node pkg="tf" type="static_transform_publisher" name="base_link_to_laser_back" args="-0.28 0.0 0.21 3.14 0 0 /base_link /laser_back 40" />
 */
const std::string TARGET_SCANNER_FRAME = "laser";

/*
 * Setting up the topic names of the scanners which should be merged
 */
const std::string FRONT_SCAN_TOPIC = "scan";
const std::string BACK_SCAN_TOPIC = "scan_back";

class Combined_Scanner {
     public:
        Combined_Scanner();
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
        
        bool new_back_scan;
        
        /**
         * Parameters of the scanners
         */
        std::string front_frame_id;
        float front_range_min;
        float front_range_max;
        ros::Time front_time;
        
        std::string back_frame_id;
        float back_range_min;
        float back_range_max;
        ros::Time back_time;
        
        /*
         * The received scans will be saved into PointCloud2
         * This data type is better for merging
         */
        sensor_msgs::PointCloud2 front_scan_cloud;
        sensor_msgs::PointCloud2 back_scan_cloud;
        
        void sendScan();
};

Combined_Scanner::Combined_Scanner(){
    new_back_scan = false;
    
    // Subscribe the scan topics of the both scanners
    front_scan_sub_ = node_.subscribe<sensor_msgs::LaserScan> ("/" + FRONT_SCAN_TOPIC, 1000, &Combined_Scanner::frontScanCallback, this);
    back_scan_sub_ = node_.subscribe<sensor_msgs::LaserScan> ("/" + BACK_SCAN_TOPIC, 1000, &Combined_Scanner::backScanCallback, this);
    
    // Topic for publishing the combined scanner
    point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2> ("/" + COMBINED_LASER_TOPIC, 1000, false);
    
    tfListener_front_.setExtrapolationLimit(ros::Duration(0.1));
    tfListener_back_.setExtrapolationLimit(ros::Duration(0.1));
}

void Combined_Scanner::frontScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    ROS_INFO("Received front scan");
    
    front_frame_id = scan->header.frame_id;
    front_range_min = scan->range_min;
    front_range_max = scan->range_max;
    front_time = scan->header.stamp;
    
    // transform the scan information into the frame of the combined scanner
    bool result_transformation = tfListener_front_.waitForTransform(front_frame_id, "/" + TARGET_SCANNER_FRAME, 
                                                                    scan->header.stamp + ros::Duration().fromSec(scan->ranges.size()*scan->time_increment),
                                                                    ros::Duration(1.0)
                                                                    );
    
    if(!result_transformation){
        ROS_WARN("The transformation into the target frame failed.");
        return;
    }
    
    try {
		// Convert the distance information into a pointcloud
        projector_front_.transformLaserScanToPointCloud("/" + TARGET_SCANNER_FRAME, *scan, front_scan_cloud, tfListener_front_);
    } catch (int err) {
		ROS_ERROR("The transformation into the point cloud failed.");
	}
    
    /*
     * Publish instant?
     * No good experience
     */
    // point_cloud_publisher_.publish(front_scan_cloud);
    
    sendScan();
}

void Combined_Scanner::backScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    ROS_INFO("Received back scan");
    
    back_frame_id = scan->header.frame_id;
    back_range_min = scan->range_min;
    back_range_max = scan->range_max;
    back_time = scan->header.stamp;
    
    // transform the scan information into the frame of the combined scanner
    bool result_transformation = tfListener_back_.waitForTransform(back_frame_id, "/" + TARGET_SCANNER_FRAME, 
                                                                   scan->header.stamp + ros::Duration().fromSec(scan->ranges.size()*scan->time_increment),
                                                                   ros::Duration(1.0)
                                                                   );
    
    if(!result_transformation){
        ROS_WARN("The transformation into the target frame failed.");
        return;
    }
    
    try {
		// Convert the distance information into a pointcloud
        projector_back_.transformLaserScanToPointCloud("/" + TARGET_SCANNER_FRAME, *scan, back_scan_cloud, tfListener_back_);
    } catch (int err) {
		ROS_ERROR("The transformation into the point cloud failed.");
	}
    
    /*
     * PUBLISH ALWAYS IF A NEW BACK SCAN IS AVAIBLE:
     * no good experience
     */
    // point_cloud_publisher_.publish(back_scan_cloud);
    new_back_scan = true;
}

void Combined_Scanner::sendScan(){
	// only process if new back scan signal was received
	// otherwise multiple front scan signals would be merged with the same back scan signal
    if (new_back_scan) {
        // check time difference
        ros::Duration time_difference_s = front_time - back_time; // could be negative
        double raw_time_difference_s = abs(time_difference_s.toSec());
        
        // only accepts 0.02s difference
        if(raw_time_difference_s <= 0.02) {
            // combine clouds of the front and back scanner
            sensor_msgs::PointCloud2 combined_scan;
            pcl::concatenatePointCloud(front_scan_cloud, back_scan_cloud, combined_scan);
            point_cloud_publisher_.publish(combined_scan);
        }
        new_back_scan = false;
    } else {
		// --> publish it without back scan information (better then nothing)
        point_cloud_publisher_.publish(front_scan_cloud);
    }
}

int main(int argc, char** argv)
{
    ROS_INFO("Starting node for publishing a combined laser scanner.");
    ros::init(argc, argv, "two_scan_publisher");

    // Start combined laser scanner
    Combined_Scanner cScanner;
    ROS_INFO("Combined laser scanner was started.");
    
    // Prevent closing the program
    ros::spin();

    return 0;
}
