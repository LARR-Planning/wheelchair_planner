//
// Created by larr-desktop on 22. 10. 21.
//
#include <wheelchair_planner/SingleBodyPlanner.h>

SingleBodyPlanner::SingleBodyPlanner(const ParamSingle &s_param, shared_ptr<PlannerBase> p_base_): AbstractPlanner(p_base_),param(s_param) {
    std::cout<<"[SINGLE BODY PLANNER]: INITIALIZATION"<<std::endl;
}

bool SingleBodyPlanner::plan(double t) {
    //TODO: IMPLEMENT PLANNING ALGORITHM HERE

    return false;
}

void SingleBodyPlanner::updateTrajToBase() {
    {
        p_base->mSet[1].lock();
        //TODO: Update Planning Result
        p_base->mSet[1].unlock();
    }
}


