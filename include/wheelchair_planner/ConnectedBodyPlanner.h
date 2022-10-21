//
// Created by larr-desktop on 22. 10. 21.
//

#ifndef WHEELCHAIR_PLANNER_CONNECTEDBODYPLANNER_H
#define WHEELCHAIR_PLANNER_CONNECTEDBODYPLANNER_H
#include <wheelchair_planner/PlannerCore.h>
using namespace Planner;

class ConnectedBodyPlanner: public AbstractPlanner{
private:
    ParamConnected param;
public:
    ConnectedBodyPlanner(const ParamConnected &c_param, shared_ptr<PlannerBase>p_base_);
    bool plan(double t) override;
    void updateTrajToBase()  override;
    void updateValue() override;

};


#endif //WHEELCHAIR_PLANNER_CONNECTEDBODYPLANNER_H
