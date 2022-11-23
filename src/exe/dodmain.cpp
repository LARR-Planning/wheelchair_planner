//
// Created by larr-desktop on 22. 11. 23.
//
#include <dynobs_detector/DynObsDetector.h>


int main(int argc, char* argv[]){
    ros::init(argc,argv,"dynobs_detector");
    DynObsDetector det;
    det.run();
    return 0;
}