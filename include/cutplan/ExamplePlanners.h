#ifndef __CUTPLAN_EXAMPLEPLANNERS_H__

#define __CUTPLAN_EXAMPLEPLANNERS_H__

#include <cutplan/ManeuverPlanner.h>
#include <cutplan/ExampleManeuvers.h>

namespace cutplan
{

class ColoringTask: public ManeuverPlannerNoRegularization
{
public:
    ColoringTask():
        ManeuverPlannerNoRegularization()
    {
        nextManeuver.reset(new Brush());
    }
    ~ColoringTask();
    virtual bool planNextManeuver(ManeuverConstPtr &nextManeuverP);
};

class GrindingTask: public ManeuverPlannerNoRegularization
{
public:
    GrindingTask():
        ManeuverPlannerNoRegularization()
    {
        nextManeuver.reset(new Burr());
    }
    ~GrindingTask();
    virtual bool planNextManeuver(ManeuverConstPtr &nextManeuverP);
};

class SweepingTask: public ManeuverPlannerConvexRegularization
{
public:
    SweepingTask():
        ManeuverPlannerConvexRegularization()
    {
        nextManeuver.reset(new Wiper());
    }
    ~SweepingTask();
    virtual bool planNextManeuver(ManeuverConstPtr &nextManeuverP);
};

}

#endif
