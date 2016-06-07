#ifndef __CUTPLAN_SAMPLINGVOLUME_H__

#define __CUTPLAN_SAMPLINGVOLUME_H__

#include <cutplan/typedefs.h>

#include <CGAL/Side_of_triangle_mesh.h>

#include <ompl/util/RandomNumbers.h>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

namespace cutplan
{

class VolumeSample;
class VomlumeSampler;
class SamplingVolume;

class VolumeSample: public ompl::base::RealVectorStateSpace::StateType
{
public:
    void set(tPointSpec const&p)
    {
        values[0] = ::CGAL::to_double(p.x());
        values[1] = ::CGAL::to_double(p.y());
        values[2] = ::CGAL::to_double(p.z());
    }
    void get(tPointSpec &p) const
    {
        p = tPointSpec(values[0], values[1], values[2]);
    }
};

class SamplingVolume: public ompl::base::RealVectorStateSpace
{
public:
    typedef VolumeSample StateType;
    SamplingVolume():
        ompl::base::RealVectorStateSpace(3)
    {
        inited = false;
        insideQuery = 0;
        intersectionQuery = 0;
        densitySum = 0.0;
        type_ = 7013;
    }
    SamplingVolume(SamplingVolume const& orig):
        ompl::base::RealVectorStateSpace(3)
    {
        inited = false;
        insideQuery = 0;
        intersectionQuery = 0;
        densitySum = 0.0;
        type_ = 7013;
        *this = orig;
    }
    ~SamplingVolume(){}

    SamplingVolume& operator=(SamplingVolume const& orig);

    virtual unsigned int getDimension(void) const;
    virtual double getMaximumExtent(void) const;
    virtual double getMeasure(void) const;
    virtual ompl::base::StateSamplerPtr allocDefaultStateSampler(void) const;
    virtual ompl::base::State* allocState() const;
    bool getVertexSample(int n, std::vector<tPointSpec> &samples) const;
    bool getVertices(std::vector<tPointSpec> & samples) const;
    bool updateProbabilityDensity(tPointSpec const& P, double a, double r);
    bool resetProbabilityDensity(void);
    bool inVolume(tPointSpec const& P) const;
    bool intersectsSegment(tPointSpec const& A, tPointSpec const& B) const;
    bool closestPoint2Point(tPointSpec const& P, tPointSpec &R) const;

    bool update(meshproc_csg::MeshEntry const& v);
    bool update(MeshEntryConstPtr const& v);
    bool update(meshproc_csg::MeshEntry const& v, MeshEntryConstPtr const& gPlus, MeshEntryConstPtr const& gMinus);
    bool update(MeshEntryConstPtr const& v, MeshEntryConstPtr const& gPlus, MeshEntryConstPtr const& gMinus);
    bool removeOtherSamplingVolume(boost::shared_ptr<SamplingVolume const> const& volumeP);
    bool removeOtherSamplingVolume(meshproc_csg::MeshEntry const& volumeP);
    meshproc_csg::MeshEntry const& getMeshEntry(void) const
    {
        return volume;
    }

    bool isInited(void) const {return inited;}

    static double PSpecDistance(tPointSpec const& A, tPointSpec const& B);
    static tPointSpec PSpecCentroid(std::vector<tPointSpec> const& samples);

protected:
    bool updateAuxiliaries(void);
    int selectRandomSample(void) const;
    tPointSpec getVertex(int index) const;

    meshproc_csg::MeshEntry volume;
    CGAL::Side_of_triangle_mesh<meshproc_csg::Polyhedron, meshproc_csg::Kernel> *insideQuery;
    typedef CGAL::AABB_face_graph_triangle_primitive<meshproc_csg::Polyhedron> AABBPrimitive;
    typedef CGAL::AABB_traits<meshproc_csg::Kernel, AABBPrimitive> AABBTraits;
    typedef meshproc_csg::Kernel::Segment_3 Segment;
    CGAL::AABB_tree<AABBTraits> *intersectionQuery;
    std::vector<double> density;
    std::vector<double> accumulatedDensity;
    std::vector<tPointSpec> vertices;
    double densitySum;
    bool inited;
    mutable ompl::RNG rng;
};

class VolumeSampler: public ompl::base::RealVectorStateSampler
{
public:
    VolumeSampler(const SamplingVolume *space):
        RealVectorStateSampler(space)
    {}
    virtual void sampleUniform(ompl::base::State *state);
    virtual void sampleUniformNear(ompl::base::State *state, const ompl::base::State *near, const double distance);
    virtual void sampleGaussian(ompl::base::State *state, const ompl::base::State *mean, const double stdDev);
protected:
    bool stateInVolume(VolumeSample *state) const;
};

typedef boost::shared_ptr<SamplingVolume> SamplingVolumePtr;
typedef boost::shared_ptr<SamplingVolume const> SamplingVolumeConstPtr;

}

#endif
