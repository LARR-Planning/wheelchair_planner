//
// Created by larr-desktop on 22. 10. 21.
//
#include <wheelchair_planner/ConnectedBodyPlanner.h>

ConnectedBodyPlanner::ConnectedBodyPlanner(const ParamConnected &c_param, shared_ptr<PlannerBase> p_base_): AbstractPlanner(p_base_),param(c_param) {
    std::cout<<"[CONNECTED BODY PLANNER]: INITIALIZATION"<<std::endl;
}

bool ConnectedBodyPlanner::plan(double t) {
    //TODO: IMPLEMENT PLANNING ALGORITHM HERE;

    return false;
}

void ConnectedBodyPlanner::updateTrajToBase() {
    {
        p_base->mSet[1].lock();
        //TODO: Update Planning Result
        p_base->mSet[1].unlock();
    }
}
