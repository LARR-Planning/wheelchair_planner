//
// Created by larr-desktop on 22. 10. 21.
//
#include <wheelchair_planner/Wrapper.h>

void Wrapper::run() {
    threadPlanner = thread(&Wrapper::runPlanning,this);
    std::cout<<"Thread for Planner has been created."<<std::endl;
    threadRosWrapper = thread(&RosWrapper::runROS, ros_wrapper_ptr);
    std:;cout<<"Thread for RosWrapper has been created."<<std::endl;
    threadRosWrapper.join();
    threadPlanner.join();
}

void Wrapper::runPlanning() {
    ros::Rate loop_rate_wrapper(10.0);
    bool isPlanSuccess = false;
    bool isPlanPossible = false;
    while(ros::ok()){
        isPlanPossible = ros_wrapper_ptr->isAllSourceReceived();
        if(isPlanPossible)
        {
            if(param.is_single_body_planner){
                isPlanSuccess = planSingleBody(ros_wrapper_ptr->curTime());
            }
            else{ // connected body planner
                isPlanSuccess = planConnectedBody(ros_wrapper_ptr->curTime());
            }
        }
        ros::spinOnce();
        loop_rate_wrapper.sleep();
    }
}

Wrapper::Wrapper():p_base_shared(make_shared<PlannerBase>()) {
    std::cout<<"Wrapper Executed."<<std::endl;
    ros_wrapper_ptr = new RosWrapper(p_base_shared);
    ros_wrapper_ptr->updateParam(param);
    if(param.is_single_body_planner){
        sbp_ptr = new SingleBodyPlanner(param.s_param,p_base_shared);
    }
    else // connected body planner
    {
       cbp_ptr = new ConnectedBodyPlanner(param.c_param,p_base_shared);
    }
}

bool Wrapper::planSingleBody(double tTrigger) {
    bool sbPassed = sbp_ptr->plan(tTrigger);
    if(not p_base_shared->isPlanningSolved){
        p_base_shared->isPlanningSolved = sbPassed;
    }
    if(p_base_shared->isPlanningSolved){
        updatePlanResultToBase();
    }
    return sbPassed;
}

bool Wrapper::planConnectedBody(double tTrigger) {
    bool cbPassed = cbp_ptr->plan(tTrigger);
    if(not p_base_shared->isPlanningSolved){
        p_base_shared->isPlanningSolved = cbPassed;
    }
    if(p_base_shared->isPlanningSolved){
        updatePlanResultToBase();
    }
    return cbPassed;
}

void Wrapper::updatePlanResultToBase() {
    if(param.is_single_body_planner){ //Update Planning Results of Single Body Planner
        sbp_ptr->updateTrajToBase();
    }
    else{   //Update Planning Results of Connected Body Planner
        cbp_ptr->updateTrajToBase();
    }
}

void RosWrapper::runROS() {
    ros::Rate loop_rate_rosWrapper(50.0); // publish and spin
    ros::AsyncSpinner spinner(4);
    spinner.start();
    while(ros::ok()){
        prepareROSmsgs();
        publish();
        ros::spinOnce();
        loop_rate_rosWrapper.sleep();
    }
}

void RosWrapper::prepareROSmsgs() {
    //TODO : PREPARE ROS MSGS
    {
        /**
         * PlanningReulst_ROS (datatype: ROS) <-- Planning Result (arbitrary type)
         */
    }

}

void RosWrapper::publish() {
    //TODO: PUBLISH MSGS
    {
     /**
      * publisherName.publish(PlanningResult_ROS);
      */
    }
}

RosWrapper::RosWrapper(shared_ptr<PlannerBase> p_base_):p_base(p_base_),nh("~") {
    //TODO: IMPLEMENT ROS-related stuff...
    /**
     * ROS SUBSCRIBER
     * nh.subscribe("topic_name",callback_function,this);
     */

    /**
     * ROS PUBLISHER
     * nh.advertise<datatype>("topic_name",1);
     */
}

void RosWrapper::updateParam(Param &param_) {
    nh.param<bool>("is_single_body",param.is_single_body_planner,true);
    if(param.is_single_body_planner){ //Params for SingleBodyPlanner
        ROS_INFO("Parameters for Single Body Planner Updated.");
        nh.param<double>("dummy_value1",param.s_param.dummy_value1,1.0);
    }
    else{   //Params for ConnectedBodyPlanner
        ROS_INFO("Parameters for Connected Body Planner Updated.");
        nh.param<double>("dummy_value2",param.c_param.dummy_value2,2.0);
    }
    t0 = ros::Time::now().toSec();
}

bool RosWrapper::isAllSourceReceived() {
    //TODO: ADD CONDITION TO DETERMINE START
    return false;
}
