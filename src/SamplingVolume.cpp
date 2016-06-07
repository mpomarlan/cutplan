#include <cutplan/typedefs.h>

#include <cutplan/SamplingVolume.h>

#include <malloc.h>

namespace cutplan
{

ompl::base::State* SamplingVolume::allocState() const
{
    SamplingVolume::StateType *retq = new VolumeSample();
    retq->values = (double*)malloc(3*sizeof(double));

    retq->values[0] = 0;
    retq->values[1] = 0;
    retq->values[2] = 0;
    return retq;
}

void VolumeSampler::sampleUniform(ompl::base::State *state)
{
    VolumeSample *vstate = static_cast<VolumeSample *>(state);
    bool haveState = false;
    do
    {
        RealVectorStateSampler::sampleUniform(vstate);
        haveState = stateInVolume(vstate);
    }while(!haveState);
}
void VolumeSampler::sampleUniformNear(ompl::base::State *state, const ompl::base::State *near, const double distance)
{
    VolumeSample *vstate = static_cast<VolumeSample *>(state);
    bool haveState = false;
    do
    {
        RealVectorStateSampler::sampleUniformNear(vstate, near, distance);
        haveState = stateInVolume(vstate);
    }while(!haveState);
}
void VolumeSampler::sampleGaussian(ompl::base::State *state, const ompl::base::State *mean, const double stdDev)
{
    VolumeSample *vstate = static_cast<VolumeSample *>(state);
    bool haveState = false;
    do
    {
        RealVectorStateSampler::sampleGaussian(vstate, mean, stdDev);
        haveState = stateInVolume(vstate);
    }while(!haveState);
}
bool VolumeSampler::stateInVolume(VolumeSample *state) const
{
    if((!space_) || (!static_cast<SamplingVolume const*>(space_)->isInited()))
        return false;
    tPointSpec p;
    state->get(p);
    return static_cast<SamplingVolume const*>(space_)->inVolume(p);
}


SamplingVolume& SamplingVolume::operator=(SamplingVolume const& orig)
{
    inited = orig.inited;
    volume.setFromMeshEntry(orig.volume);
    density = orig.density;
    densitySum = orig.densitySum;
    accumulatedDensity = orig.accumulatedDensity;
    if(insideQuery)
        delete insideQuery;
    if(intersectionQuery)
        delete intersectionQuery;
    insideQuery = new CGAL::Side_of_triangle_mesh<meshproc_csg::Polyhedron, meshproc_csg::Kernel>(volume.getMesh());
    intersectionQuery = new CGAL::AABB_tree<AABBTraits>(CGAL::faces(volume.getMesh()).first, CGAL::faces(volume.getMesh()).second, volume.getMesh());
}

unsigned int SamplingVolume::getDimension(void) const
{
    return 3;
}
double SamplingVolume::getMaximumExtent(void) const
{
    double xm, xM, ym, yM, zm, zM;
    volume.getBoundingBox(xM, xm, yM, ym, zM, zm);
    double dx = xM - xm;
    double dy = yM - ym;
    double dz = zM - zm;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}
double SamplingVolume::getMeasure(void) const
{
    double xm, xM, ym, yM, zm, zM;
    volume.getBoundingBox(xM, xm, yM, ym, zM, zm);
    double dx = xM - xm;
    double dy = yM - ym;
    double dz = zM - zm;
    return dx*dy*dz;
}
ompl::base::StateSamplerPtr SamplingVolume::allocDefaultStateSampler(void) const
{
    return ompl::base::StateSamplerPtr(new VolumeSampler(this));
}

bool SamplingVolume::getVertexSample(int n, std::vector<tPointSpec> &samples) const
{
    if(!inited)
        return false;
    std::vector<int> ks;
    ks.resize(n);
    samples.resize(n);
    for(int k = 0; k < n; k++)
        ks[k] = selectRandomSample();
    for(int k = 0; k < n; k++)
        samples[k] = getVertex(ks[k]);
    return true;
}

tPointSpec SamplingVolume::getVertex(int index) const
{
    if((!inited) || (vertices.size() <= index))
        return tPointSpec(0, 0, 0);
    return vertices[index];
}

bool SamplingVolume::getVertices(std::vector<tPointSpec> & samples) const
{
    if(!inited)
        return false;
    samples = vertices;
    return true;
}

double SamplingVolume::PSpecDistance(tPointSpec const& A, tPointSpec const& B)
{
    double ax, ay, az, bx, by, bz;
    ax = ::CGAL::to_double(A.x());
    ay = ::CGAL::to_double(A.y());
    az = ::CGAL::to_double(A.z());
    bx = ::CGAL::to_double(B.x());
    by = ::CGAL::to_double(B.y());
    bz = ::CGAL::to_double(B.z());
    double dx, dy, dz;
    dx = ax - bx;
    dy = ay - by;
    dz = az - bz;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

tPointSpec SamplingVolume::PSpecCentroid(std::vector<tPointSpec> const& samples)
{
    double x, y, z;
    x = 0;
    y = 0;
    z = 0;
    int maxK = samples.size();
    for(int k = 0; k < maxK; k++)
    {
        x += ::CGAL::to_double(samples[k].x());
        y += ::CGAL::to_double(samples[k].y());
        z += ::CGAL::to_double(samples[k].z());
    }
    if(maxK)
    {
        x /= (1.0*maxK);
        y /= (1.0*maxK);
        z /= (1.0*maxK);
    }
    return tPointSpec(x, y, z);
}

bool SamplingVolume::updateProbabilityDensity(tPointSpec const& P, double a, double r)
{
    if(!inited)
        return false;
    int maxK = vertices.size();
    density.resize(maxK);
    accumulatedDensity.resize(maxK);
    for(int k = 0; k < maxK; k++)
    {
        density[k] += (a/(1 + (PSpecDistance(P, vertices[k])/r)));
        if(density[k] < 0)
            density[k] = 0;
    }
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

bool SamplingVolume::inVolume(tPointSpec const& P) const
{
    if(!inited)
        return false;
    return true;
    CGAL::Bounded_side res = (*insideQuery)(P);
    return ((res == CGAL::ON_BOUNDARY) || (res == CGAL::ON_BOUNDED_SIDE));
}
bool SamplingVolume::intersectsSegment(tPointSpec const& A, tPointSpec const& B) const
{
    if(!inited)
        return false;
    Segment segmentQuery(A, B);
    //We will tolerate segments that start or end on the boundary of a space (but not both, in this implementation)
    return (intersectionQuery->do_intersect(segmentQuery) && (1 < intersectionQuery->number_of_intersected_primitives(segmentQuery)));
}
bool SamplingVolume::closestPoint2Point(tPointSpec const& P, tPointSpec &R) const
{
    if(!inited)
        return false;
    R = intersectionQuery->closest_point(P);
    return true;
}

bool SamplingVolume::removeOtherSamplingVolume(boost::shared_ptr<SamplingVolume const> const& volumeP)
{
    if(!inited)
        return false;

    if(!volumeP->inited)
        return false;
    volume.setFromDifference(volume, volumeP->volume);
    return true;
}
bool SamplingVolume::removeOtherSamplingVolume(meshproc_csg::MeshEntry const& volumeP)
{
    if(!inited)
        return false;

    volume.setFromDifference(volume, volumeP);
    return true;
}

bool SamplingVolume::update(meshproc_csg::MeshEntry const& v)
{
    volume.setFromMeshEntry(v);
    inited = true;
    return updateAuxiliaries();
}
bool SamplingVolume::update(MeshEntryConstPtr const &v)
{
    return update(*(v.get()));
}
bool SamplingVolume::update(meshproc_csg::MeshEntry const& v, MeshEntryConstPtr const& gPlus, MeshEntryConstPtr const& gMinus)
{
    volume.setFromMeshEntry(v);
    if(gPlus.get())
        volume.setFromUnion(volume, *gPlus);
    if(gMinus.get())
        volume.setFromDifference(volume, *gMinus);
    inited = true;
    return updateAuxiliaries();
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
    int maxK = volume.getNrVertices();
    if(!maxK)
    {
        inited = false;
        return false;
    }
    double xM, xm, yM, ym, zM, zm;
    volume.getVertices(vertices);
    volume.getBoundingBox(xM, xm, yM, ym, zM, zm);
    ompl::base::RealVectorBounds bounds(3);
    bounds.setLow(0, xm); bounds.setHigh(0, xM);
    bounds.setLow(1, ym); bounds.setHigh(1, yM);
    bounds.setLow(2, zm); bounds.setHigh(2, zM);
    setBounds(bounds);
    if(insideQuery)
        delete insideQuery;
    insideQuery = new CGAL::Side_of_triangle_mesh<meshproc_csg::Polyhedron, meshproc_csg::Kernel>(volume.getMesh());
    if(intersectionQuery)
        delete intersectionQuery;
    intersectionQuery = new CGAL::AABB_tree<AABBTraits>(CGAL::faces(volume.getMesh()).first, CGAL::faces(volume.getMesh()).second, volume.getMesh());
    density.resize(maxK);
    accumulatedDensity.resize(maxK);
    resetProbabilityDensity();
    return true;
}

int SamplingVolume::selectRandomSample(void) const
{
    double r = rng.uniformReal(0, densitySum);
    int l = 0;
    int m = accumulatedDensity.size() - 1;
    while(l != m)
    {
        int h = ((l + m)>>1);
        double q = accumulatedDensity[h];
        if(q < r)
            l = h + 1;
        else
            m = h;
    }
    return l;
}

}
