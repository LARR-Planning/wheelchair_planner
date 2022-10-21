//
// Created by larr-desktop on 22. 10. 21.
//
#ifndef WHEELCHAIR_PLANNER_WRAPPER_H
#define WHEELCHAIR_PLANNER_WRAPPER_H


#include <wheelchair_planner/ConnectedBodyPlanner.h>
#include <wheelchair_planner/SingleBodyPlanner.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>


class RosWrapper{
private:
    double t0;
    ros::NodeHandle nh;
    shared_ptr<PlannerBase> p_base;
    Param param;

    //TODO: ROS SUBSCRIBER DECLARATION

    //TODO: CALLBACK FUNCTION DECLARATION

    //TODO: ROS PUBLISHER DECLARATION


    void prepareROSmsgs();
    void publish();

public:
    RosWrapper(shared_ptr<PlannerBase> p_base_);
    void updateParam(Param &param_);
    void runROS();
    double curTime(){return (ros::Time::now().toSec()-t0);}
    bool isAllSourceReceived();

};


class Wrapper{
private:
    shared_ptr<PlannerBase> p_base_shared;
    thread threadPlanner;
    thread threadRosWrapper;

    Param param;
    SingleBodyPlanner* sbp_ptr;
    ConnectedBodyPlanner *cbp_ptr;
    RosWrapper* ros_wrapper_ptr;

    bool planSingleBody(double tTrigger);
    bool planConnectedBody(double tTrigger);

    void runPlanning();
    void updatePlanResultToBase();

public:
    Wrapper();
    void run();

};




#endif //WHEELCHAIR_PLANNER_WRAPPER_H
