#ifndef __MANEUVER_H__

#define __MANEUVER_H__

#include <cutplan/typedefs.h>
#include <meshproc_csg/csg.h>

namespace cutplan
{

class Maneuver
{
public:
    Maneuver();
    Maneuver(Maneuver const& orig);
    ~Maneuver();

    Maneuver& operator=(Maneuver const& orig);

    virtual bool setFromSample(std::vector<tPointSpec> const& samples) = 0;
    virtual bool setFromSample(std::vector<tPointSpec> const& samples, meshproc_csg::MeshEntry const& collector) = 0;
    bool filterForbidden(meshproc_csg::MeshEntry const& forbidden);
    bool filterForbidden(meshproc_csg::MeshEntry const& forbidden, meshproc_csg::MeshEntry const& collector);
    MeshEntryConstPtr getGoalPlus(void) const;
    MeshEntryConstPtr getGoalMinus(void) const;
    std::vector<double> getParameters(void) const;

protected:
    MeshEntryPtr goalPlus, goalMinus, volume;
    std::vector<double> parameters;
    bool inited;
};
}

#endif
