//
// Created by larr-desktop on 22. 11. 23.
//

#ifndef WHEELCHAIR_PLANNER_DYNOBSDETECTOR_H
#define WHEELCHAIR_PLANNER_DYNOBSDETECTOR_H

#include<ros/ros.h>
#include<string>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <std_msgs/Bool.h>


class DynObsDetector{
private:
    ros::NodeHandle nh;
    ros::Subscriber subPcl;
    ros::Publisher pubConsideredPcl;
    ros::Publisher pubDynExist;
    void cbPcl(const sensor_msgs::PointCloud2::ConstPtr& pcl_msgs);

    std::string robot_frame_id;
    double detect_range_lateral;
    double detect_range_longitudinal;
    int num_threshold_stop;

    sensor_msgs::PointCloud pcl_msg_converted;
    sensor_msgs::PointCloud pcl_considered;

    void pclExtraction();
    void determineStop();
    void publish();
    std_msgs::Bool isDynObs;
public:
    DynObsDetector();
    void run();
};


#endif //WHEELCHAIR_PLANNER_DYNOBSDETECTOR_H
