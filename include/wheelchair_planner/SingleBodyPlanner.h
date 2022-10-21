//
// Created by larr-desktop on 22. 10. 21.
//

#ifndef WHEELCHAIR_PLANNER_SINGLEBODYPLANNER_H
#define WHEELCHAIR_PLANNER_SINGLEBODYPLANNER_H
#include <wheelchair_planner/PlannerCore.h>
using namespace Planner;
class SingleBodyPlanner: public AbstractPlanner{
private:
    ParamSingle param;
public:
    SingleBodyPlanner(const ParamSingle &s_param, shared_ptr<PlannerBase>p_base_);
    bool plan(double t) override;
    void updateTrajToBase()  override;

};



#endif //WHEELCHAIR_PLANNER_SINGLEBODYPLANNER_H
