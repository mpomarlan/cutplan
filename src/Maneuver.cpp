#include <cutplan/Maneuver.h>
#include <cutplan/SamplingVolume.h>

namespace cutplan
{

Maneuver::Maneuver()
{
    inited = false;
    goalPlus.reset();
    goalMinus.reset();
    volume.reset();
    parameters.clear();
}

Maneuver::Maneuver(Maneuver const& orig)
{
    inited = orig.inited;
    goalPlus.reset();
    goalMinus.reset();
    volume.reset();
    parameters = orig.parameters;
    if(inited)
    {
        volume.reset(new meshproc_csg::MeshEntry()); volume->setFromMeshEntry(*(orig.volume));
        if(orig.goalPlus.get())
        {
            if(orig.goalPlus.get() == orig.volume.get())
                goalPlus = this->volume;
            else
                goalPlus.reset(new meshproc_csg::MeshEntry(*(orig.volume)));
        }
        if(orig.goalMinus.get())
        {
            if(orig.goalMinus.get() == orig.volume.get())
                goalMinus = this->volume;
            else
                goalMinus.reset(new meshproc_csg::MeshEntry(*(orig.volume)));
        }
    }
}
Maneuver::~Maneuver()
{
}

Maneuver& Maneuver::operator=(Maneuver const& orig)
{
    inited = orig.inited;
    goalPlus.reset();
    goalMinus.reset();
    volume.reset();
    parameters = orig.parameters;
    if(inited)
    {
        volume.reset(new meshproc_csg::MeshEntry()); volume->setFromMeshEntry(*(orig.volume));
        if(orig.goalPlus.get())
        {
            if(orig.goalPlus.get() == orig.volume.get())
                goalPlus = this->volume;
            else
                goalPlus.reset(new meshproc_csg::MeshEntry(*(orig.volume)));
        }
        if(orig.goalMinus.get())
        {
            if(orig.goalMinus.get() == orig.volume.get())
                goalMinus = this->volume;
            else
                goalMinus.reset(new meshproc_csg::MeshEntry(*(orig.volume)));
        }
    }

    return *this;
}

bool Maneuver::filterForbidden(meshproc_csg::MeshEntry const& forbidden)
{
    /*
     *  - substract forbidden from volume <- ok
     *  - select connected component of highest volume from the result <- check that we have sep. by connected comps
     *  - sample connected component to reinit g+, g-, v, ps; <- ok-ish, needs a def. of the other class
    */
    std::vector<tPointSpec> samples;
    if(!inited)
        return false;
    volume->setFromDifference(*volume, forbidden);
    //
    return setFromSample(samples);
}
bool Maneuver::filterForbidden(meshproc_csg::MeshEntry const& forbidden, meshproc_csg::MeshEntry const& collector)
{
    /*
     *  - substract forbidden from volume <- ok
     *  - select connected component of highest volume from the result <- check that we have sep. by connected comps
     *  - sample connected component to reinit g+, g-, v, ps; <- ok-ish, needs a def. of the other class
    */
    std::vector<tPointSpec> samples;
    if(!inited)
        return false;
    volume->setFromDifference(*volume, forbidden);
    //
    return setFromSample(samples, collector);
}

MeshEntryConstPtr Maneuver::getGoalPlus(void) const
{
    return goalPlus;
}
MeshEntryConstPtr Maneuver::getGoalMinus(void) const
{
    return goalMinus;
}
std::vector<double> Maneuver::getParameters(void) const
{
    return parameters;
}

}
