#include <cutplan/typedefs.h>

#include <cutplan/SamplingVolume.h>

namespace cutplan
{

SamplingVolume::SamplingVolume()
{
    inited = false;
    densitySum = 0.0;
}

SamplingVolume::SamplingVolume(SamplingVolume const& orig)
{
    inited = orig.inited;
    volume.setFromMeshEntry(orig.volume);
    density = orig.density;
    densitySum = orig.densitySum;
    accumulatedDensity = orig.accumulatedDensity;
    //
}

SamplingVolume::~SamplingVolume()
{
    ;
}

SamplingVolume& SamplingVolume::operator=(SamplingVolume const& orig)
{
    inited = orig.inited;
    volume.setFromMeshEntry(orig.volume);
    density = orig.density;
    densitySum = orig.densitySum;
    accumulatedDensity = orig.accumulatedDensity;
    //
}

bool SamplingVolume::getVertexSample(int n, std::vector<tPointSpec> &samples) const
{
    if(!inited)
        return false;
    //
    return true;
}

bool SamplingVolume::getVolumeSample(tPointSpec & P) const
{
    if(!inited)
        return false;
    //
    return true;
}
bool SamplingVolume::updateProbabilityDensity(tPointSpec const& P, double a, double r)
{
    if(!inited)
        return false;
    //
    int maxK = density.size();
    accumulatedDensity[0] = density[0];
    for(int k = 1; k < maxK; k++)
        accumulatedDensity[k] = accumulatedDensity[k-1] + density[k];
    densitySum = accumulatedDensity[maxK-1];

    return true;
}

bool SamplingVolume::resetProbabilityDensity(void)
{
    if(!inited)
        return false;
    int maxK = density.size();
    for(int k = 0; k < maxK ; k++)
    {
        density[k] = 1;
        accumulatedDensity[k] = k + 1;
    }
    densitySum = maxK;
    return true;
}

bool SamplingVolume::intersectsSegment(tPointSpec const& A, tPointSpec const& B) const
{
    if(!inited)
        return false;
    //
    return true;
}
bool SamplingVolume::closestPoint2Point(tPointSpec const& P, tPointSpec &R) const
{
    if(!inited)
        return false;
    //
    return true;
}

bool SamplingVolume::update(MeshEntryConstPtr const& v, MeshEntryConstPtr const& gPlus, MeshEntryConstPtr const& gMinus)
{
    if(v.get())
        volume.setFromMeshEntry(*v);
    if(gPlus.get())
        volume.setFromUnion(volume, *gPlus);
    if(gMinus.get())
        volume.setFromDifference(volume, *gMinus);
    if(v.get() || gPlus.get())
        inited = true;
    return updateAuxiliaries();
}

bool SamplingVolume::updateAuxiliaries(void)
{
    if(!inited)
        return false;
    //
    int maxK = volume.getNrVertices();
    if(!maxK)
    {
        inited = false;
        return false;
    }
    density.resize(maxK);
    accumulatedDensity.resize(maxK);
    resetProbabilityDensity();
    return true;
}

}
