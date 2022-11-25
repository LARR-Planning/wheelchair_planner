//
// Created by larr-desktop on 22. 11. 23.
//
#include<dynobs_detector/DynObsDetector.h>

void DynObsDetector::run() {

    ros::Rate lr(20.0);
    while(ros::ok()){
        pclExtraction();
        determineStop();
        publish();

        ros::spinOnce();
        lr.sleep();
    }

}

DynObsDetector::DynObsDetector():nh("~") {
    nh.param<std::string>("robot_frame_id",robot_frame_id,"");
    nh.param<double>("detect_range_lateral",detect_range_lateral,1.0);
    nh.param<double>("detect_range_longitudinal", detect_range_longitudinal,2.0);
    nh.param<int>("num_threshold_stop",num_threshold_stop,100);
    pubConsideredPcl = nh.advertise<sensor_msgs::PointCloud>("pcl_considered",1);
    pubDynExist = nh.advertise<std_msgs::Bool>("is_dyn_exist",1);
    subPcl = nh.subscribe("pcl_topic",1,&DynObsDetector::cbPcl,this);
    isDynObs.data = false;
}

void DynObsDetector::cbPcl(const sensor_msgs::PointCloud::ConstPtr &pcl_msgs) {
    pcl_msg_converted.points.clear();

    if(not pcl_msgs->points.empty()){
        pcl_msg_converted.header.frame_id = pcl_msgs->header.frame_id;
        pcl_msg_converted.header.stamp = pcl_msgs->header.stamp;
        pcl_msg_converted.points = pcl_msgs->points;
    }
//    if(!pcl_msgs->fields.empty()){
//        sensor_msgs::convertPointCloud2ToPointCloud(*pcl_msgs,pcl_msg_converted);
//        pcl_msg_converted.header.frame_id = pcl_msgs->header.frame_id;
//        pcl_msg_converted.header.stamp = pcl_msgs->header.stamp;
//    }
}

void DynObsDetector::pclExtraction() {
    pcl_considered.points.clear();
    if(!pcl_msg_converted.points.empty()){
        pcl_considered.header.frame_id = pcl_msg_converted.header.frame_id;
        pcl_considered.header.stamp = pcl_msg_converted.header.stamp;
        double pt_x, pt_y, pt_z;
        for(const auto &point: pcl_msg_converted.points){
            pt_x = point.x;
            pt_y = point.y;
            pt_z = point.z;
            if(pt_x>0.3 and pt_x<detect_range_longitudinal and pt_y<detect_range_lateral and pt_y>-detect_range_lateral){
                pcl_considered.points.push_back(point);
            }
        }
    }
}

void DynObsDetector::determineStop() {
    if(!pcl_considered.points.empty()){
        if(pcl_considered.points.size()>num_threshold_stop){
            isDynObs.data = true;
        }
        else{
            isDynObs.data = false;
        }
    }
    else{
        isDynObs.data = false;
    }
    std::cout<<pcl_considered.points.size()<<std::endl;
}

void DynObsDetector::publish() {
    pubConsideredPcl.publish(pcl_considered);
    pubDynExist.publish(isDynObs);
}
