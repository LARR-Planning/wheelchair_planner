//
// Created by larr-desktop on 22. 10. 21.
//
#include <wheelchair_planner/Wrapper.h>

int main(int argc, char* argv[]){
    ros::init(argc,argv,"wheelchair_planner");
    Wrapper wheelchair_wrapper;
    wheelchair_wrapper.run();
    return 0;
}