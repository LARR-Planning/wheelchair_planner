//
// Created by larr-desktop on 22. 10. 21.
//

#ifndef WHEELCHAIR_PLANNER_PLANNERCORE_H
#define WHEELCHAIR_PLANNER_PLANNERCORE_H
#include <memory>
#include <thread>
#include <mutex>
#include <iostream>

using namespace std;

namespace Planner{
    struct ParamSingle{

    };
    struct ParamConnected{

    };
    struct Param{
        ParamSingle s_param;
        ParamConnected c_param;
        bool is_single_body_planner;
    };
    class PlannerBase{
    private:

    public:
        mutex mSet[2];
        bool isPlanningSolved = false;
    };

    class AbstractPlanner{
    private:
    protected:
        shared_ptr<PlannerBase> p_base;
    public:
        AbstractPlanner(shared_ptr<PlannerBase> p_base_):p_base(p_base_){};
        virtual bool plan(double t) = 0;
        virtual void updateTrajToBase() = 0;
    };
}



#endif //WHEELCHAIR_PLANNER_PLANNERCORE_H
