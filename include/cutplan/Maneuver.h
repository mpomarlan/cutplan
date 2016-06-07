#ifndef __MANEUVER_H__

#define __MANEUVER_H__

#include <cutplan/typedefs.h>
#include <meshproc_csg/csg.h>
#include <cutplan/SamplingVolume.h>

namespace cutplan
{

class Maneuver
{
public:
    Maneuver();
    Maneuver(Maneuver const& orig);
    ~Maneuver();

    Maneuver& operator=(Maneuver const& orig);

    bool setFromSample(std::vector<tPointSpec> const& samples, meshproc_csg::MeshEntry const& forbidden);
    bool setFromSample(std::vector<tPointSpec> const& samples, meshproc_csg::MeshEntry const& forbidden, SamplingVolumePtr const& regularizedGoal, SamplingVolumeConstPtr const& collector);
    MeshEntryConstPtr getGoalPlus(void) const;
    MeshEntryConstPtr getGoalMinus(void) const;
    std::vector<double> getParameters(void) const;
    static bool getCentroid(std::vector<tPointSpec> const& samples, tPointSpec& centroid);
    static bool getPCA(std::vector<tPointSpec> const& samples, tPointSpec& direction);
    static bool getExtension(std::vector<tPointSpec> const& samples, tPointSpec const& centroid, tPointSpec const& direction, tPointSpec& A, tPointSpec& B);
    static bool getAxes(tPointSpec const& directionX, tPointSpec const& refZ, tPointSpec &directionY, tPointSpec & directionZ);
    static bool movePoint(tPointSpec const& P, tPointSpec const& direction, double t, tPointSpec & R);

protected:
    bool findDirection(tPointSpec const& centroid, tPointSpec const& goal, SamplingVolumePtr const& regularizedGoal, tPointSpec &direction) const;
    bool filterForbidden(meshproc_csg::MeshEntry const& forbidden);
    bool filterForbidden(meshproc_csg::MeshEntry const& forbidden, SamplingVolumePtr const& regularizedGoal, SamplingVolumeConstPtr const& collector);
    virtual bool shapeFromSample(std::vector<tPointSpec> const& samples) = 0;
    virtual bool shapeFromSample(std::vector<tPointSpec> const& samples, tPointSpec const& direction) = 0;
    bool getMaxVolumeCC(meshproc_csg::MeshEntry &v) const;
    MeshEntryPtr goalPlus, goalMinus, volume;
    std::vector<double> parameters;
    bool inited;
};

typedef boost::shared_ptr<Maneuver> ManeuverPtr;
typedef boost::shared_ptr<Maneuver const> ManeuverConstPtr;

}

#endif
