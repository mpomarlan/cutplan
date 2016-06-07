#include <cutplan/ExamplePlanners.h>

namespace cutplan
{

bool ColoringTask::planNextManeuver(ManeuverConstPtr &nextManeuverP)
{
    if(!inited)
        return false;
    std::vector<tPointSpec> samples;
    goal->getVertexSample(2, samples);
    return planNextManeuverFromSamples(samples, nextManeuverP);
    //
}

bool GrindingTask::planNextManeuver(ManeuverConstPtr &nextManeuverP)
{
    if(!inited)
        return false;
    std::vector<tPointSpec> samples;
    goal->getVertexSample(1, samples);
    return planNextManeuverFromSamples(samples, nextManeuverP);
    //
}

bool SweepingTask::planNextManeuver(ManeuverConstPtr &nextManeuverP)
{
    if(!inited)
        return false;
    std::vector<tPointSpec> samples;
    goal->getVertexSample(1, samples);
    return planNextManeuverFromSamples(samples, nextManeuverP, true);
    //
}

}
