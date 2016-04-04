#ifndef __CUTPLAN_SAMPLINGVOLUME_H__

#define __CUTPLAN_SAMPLINGVOLUME_H__

#include <cutplan/typedefs.h>

namespace cutplan
{

class SamplingVolume: protected meshproc_csg::MeshEntry
{
public:
    SamplingVolume();
    SamplingVolume(SamplingVolume const& orig);
    ~SamplingVolume();

    SamplingVolume& operator=(SamplingVolume const& orig);

    bool getVertexSample(int n, std::vector<tPointSpec> &samples) const;
    bool getVolumeSample(tPointSpec & P) const;
    bool updateProbabilityDensity(tPointSpec const& P, double a, double r);
    bool resetProbabilityDensity(void);
    bool intersectsSegment(tPointSpec const& A, tPointSpec const& B) const;
    bool closestPoint2Point(tPointSpec const& P, tPointSpec &R) const;

    bool update(MeshEntryConstPtr const& v, MeshEntryConstPtr const& gPlus, MeshEntryConstPtr const& gMinus);

protected:
    bool updateAuxiliaries(void);

    meshproc_csg::MeshEntry volume;
    std::vector<double> density;
    std::vector<double> accumulatedDensity;
    double densitySum;
    bool inited;
};

}

#endif
