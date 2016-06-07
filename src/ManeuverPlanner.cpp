#include <cutplan/ManeuverPlanner.h>

namespace cutplan
{

ManeuverPlanner::ManeuverPlanner()
{
    inited = false;
}

bool ManeuverPlanner::planNextManeuverFromSamples(std::vector<tPointSpec> const& samples, ManeuverConstPtr &nextManeuverP)
{
    if(!inited)
        return false;

    if(!nextManeuver->setFromSample(samples, forbidden))
        return false;
    goal->update(MeshEntryConstPtr(), nextManeuver->getGoalPlus(), nextManeuver->getGoalMinus());
    regularizeGoal();

    nextManeuverP = nextManeuver;
    return true;
}
bool ManeuverPlanner::planNextManeuverFromSamples(std::vector<tPointSpec> const& samples, ManeuverConstPtr &nextManeuverP, bool useCollector)
{
    if(!inited)
        return false;

    if(!nextManeuver->setFromSample(samples, forbidden, regularizedGoal, collector))
        return false;
    goal->update(MeshEntryConstPtr(), nextManeuver->getGoalPlus(), nextManeuver->getGoalMinus());
    regularizeGoal();

    nextManeuverP = nextManeuver;
    return true;
}

bool ManeuverPlanner::updateGoal(meshproc_csg::MeshEntry & volume)
{
    goal->update(volume);
    if(collector->isInited())
        goal->removeOtherSamplingVolume(collector);
    goal->removeOtherSamplingVolume(forbidden);
    regularizeGoal();
    inited = true;

    return true;
}
bool ManeuverPlanner::updateCollector(meshproc_csg::MeshEntry & volume)
{
    collector->update(volume);
    if(inited)
    {
        goal->removeOtherSamplingVolume(collector);
        regularizeGoal();
    }
    return true;
}
bool ManeuverPlanner::updateForbidden(meshproc_csg::MeshEntry & volume)
{
    forbidden.setFromMeshEntry(volume);
    if(inited)
        goal->removeOtherSamplingVolume(forbidden);
    return true;
}

bool ManeuverPlanner::saveGoalMeshToMsg(shape_msgs::Mesh & msg) const
{
    if(!inited)
        return false;
    //
    return true;
}
bool ManeuverPlanner::saveGoalMeshToFile(std::string const& filename) const
{
    if(!inited)
        return false;
    //
    return true;
}

bool ManeuverPlannerNoRegularization::regularizeGoal(void)
{
    regularizedGoal.reset();
    return true;
}

bool ManeuverPlannerConvexRegularization::regularizeGoal(void)
{
    if(!inited)
        return false;
    //
    return true;
}
}
