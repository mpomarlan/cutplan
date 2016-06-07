#ifndef __CUTPLAN_MANEUVERPLANNER_H__

#define __CUTPLAN_MANEUVERPLANNER_H__

#include <cutplan/Maneuver.h>
#include <cutplan/SamplingVolume.h>
#include <shape_msgs/Mesh.h>

namespace cutplan
{

class ManeuverPlanner
{
public:
    ManeuverPlanner();
    ~ManeuverPlanner(){}

    virtual bool planNextManeuver(ManeuverConstPtr &nextManeuverP) = 0;
    bool updateGoal(meshproc_csg::MeshEntry & volume);
    bool updateCollector(meshproc_csg::MeshEntry & volume);
    bool updateForbidden(meshproc_csg::MeshEntry & volume);
    bool saveGoalMeshToMsg(shape_msgs::Mesh & msg) const;
    bool saveGoalMeshToFile(std::string const& filename) const;

protected:
    bool planNextManeuverFromSamples(std::vector<tPointSpec> const& samples, ManeuverConstPtr &nextManeuverP);
    bool planNextManeuverFromSamples(std::vector<tPointSpec> const& samples, ManeuverConstPtr &nextManeuverP, bool useCollector);
    virtual bool regularizeGoal(void) = 0;
    ManeuverPtr nextManeuver;
    SamplingVolumePtr goal;
    SamplingVolumePtr collector;
    SamplingVolumePtr regularizedGoal;
    meshproc_csg::MeshEntry forbidden;
    bool inited;
};

class ManeuverPlannerNoRegularization: public ManeuverPlanner
{
public:
    ~ManeuverPlannerNoRegularization(){}
protected:
    virtual bool regularizeGoal(void);
};

class ManeuverPlannerConvexRegularization: public ManeuverPlanner
{
public:
    ~ManeuverPlannerConvexRegularization(){}
protected:
    virtual bool regularizeGoal(void);
};

}
#endif
