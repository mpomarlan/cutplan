#include <cutplan/Maneuver.h>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include <CGAL/linear_least_squares_fitting_3.h>
#include <CGAL/Simple_cartesian.h>

#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/connected_components.h>
#include <CGAL/boost/graph/graph_traits_Polyhedron_3.h>

#include <shape_msgs/Mesh.h>

#include <boost/function_output_iterator.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/foreach.hpp>
#include <map>

namespace cutplan
{

typedef meshproc_csg::Polyhedron::Facet_const_handle face_descriptor;

face_descriptor findCC(face_descriptor const& f, std::map<face_descriptor, face_descriptor> &auxP)
{
    std::map<face_descriptor, face_descriptor>::iterator p = auxP.find(f);
    if(p->first != p->second)
    {
        face_descriptor tp = findCC(p->second, auxP);
        p->second = tp;
    }
    return p->second;
}

void uniteCC(face_descriptor const& A, face_descriptor const& B, std::map<face_descriptor, face_descriptor> &auxP, std::map<face_descriptor, std::size_t> &auxR, std::map<face_descriptor, std::size_t> &auxI)
{
    face_descriptor pA = findCC(A, auxP);
    face_descriptor pB = findCC(B, auxP);
    if(pA == pB)
        return;
    std::size_t rA = auxR.find(pA)->second;
    std::size_t rB = auxR.find(pB)->second;
    std::size_t iA = auxI.find(pA)->second;
    std::size_t iB = auxI.find(pB)->second;
    if(rA < rB)
    {
        auxP.find(pA)->second = pB;
        auxI.find(pA)->second = iB;
    }
    else if(rA > rB)
    {
        auxP.find(pB)->second = pA;
        auxI.find(pB)->second = iA;
    }
    else
    {
        auxP.find(pB)->second = pA;
        auxI.find(pB)->second = iA;
        auxR.find(pA)->second++;
    }
}

std::size_t getCCs(meshproc_csg::Polyhedron const& P, std::map<face_descriptor, std::size_t> &fcm)
{
    std::map<face_descriptor, face_descriptor> auxP;
    std::map<face_descriptor, std::size_t> auxR;
    std::map<face_descriptor, std::size_t> auxI;
    std::size_t maxK = 0;
    auxP.clear(); auxR.clear(); fcm.clear();
    for(meshproc_csg::Polyhedron::Facet_const_iterator it = P.facets_begin();
        it != P.facets_end(); it++)
    {
        fcm.insert(std::pair<face_descriptor, std::size_t>(it, maxK));
        auxP.insert(std::pair<face_descriptor, face_descriptor>(it, it));
        auxR.insert(std::pair<face_descriptor, std::size_t>(it, 0));
        auxI.insert(std::pair<face_descriptor, std::size_t>(it, maxK));
        maxK++;
    }
    for(meshproc_csg::Polyhedron::Halfedge_const_iterator it = P.halfedges_begin();
        it != P.halfedges_end(); it++)
    {
        meshproc_csg::Polyhedron::Facet_const_handle f, fo;
        f = it->facet();
        fo = it->opposite()->facet();
        uniteCC(f, fo, auxP, auxR, auxI);
    }

    for(std::map<face_descriptor, face_descriptor>::iterator it = auxP.begin();
        it != auxP.end(); it++)
        fcm.find(it->first)->second = auxI.find(it->second)->second;
    std::map<std::size_t,std::size_t> xidx;
    xidx.clear();
    for(std::map<face_descriptor, std::size_t>::iterator it = fcm.begin();
        it != fcm.end(); it++)
    {
        if(xidx.end() == xidx.find(it->second))
            xidx.insert(std::pair<std::size_t, std::size_t>(it->second, xidx.size()));
        it->second = xidx.find(it->second)->second;
    }

    return xidx.size();
}

double getSignedTriangleVolume(meshproc_csg::Polyhedron::Facet const& f)
{
    meshproc_csg::Polyhedron::Halfedge_around_facet_const_circulator hc_a = f.facet_begin();
    meshproc_csg::Polyhedron::Halfedge_around_facet_const_circulator hc_b = hc_a; hc_b++;
    meshproc_csg::Polyhedron::Halfedge_around_facet_const_circulator hc_c = hc_b; hc_c++;

    double xA, yA, zA;
    double xB, yB, zB;
    double xC, yC, zC;

    xA = ::CGAL::to_double(hc_a->vertex()->point().x());
    yA = ::CGAL::to_double(hc_a->vertex()->point().y());
    zA = ::CGAL::to_double(hc_a->vertex()->point().z());

    xB = ::CGAL::to_double(hc_b->vertex()->point().x());
    yB = ::CGAL::to_double(hc_b->vertex()->point().y());
    zB = ::CGAL::to_double(hc_b->vertex()->point().z());

    xC = ::CGAL::to_double(hc_c->vertex()->point().x());
    yC = ::CGAL::to_double(hc_c->vertex()->point().y());
    zC = ::CGAL::to_double(hc_c->vertex()->point().z());

    double v321 = xC*yB*zA;
    double v231 = xB*yC*zA;
    double v312 = xC*yA*zB;
    double v132 = xA*yC*zB;
    double v213 = xB*yA*zC;
    double v123 = xA*yB*zC;

    return (1.0/6.0)*(-v321 + v231 + v312 - v132 - v213 + v123);
}


bool trivialStateChecker(const ompl::base::State *state)
{
     return true;
}
class R3SegmentValidator: public ompl::base::MotionValidator
{
public:
    R3SegmentValidator(ompl::base::SpaceInformation *si):
        MotionValidator(si)
    {}
    R3SegmentValidator(const ompl::base::SpaceInformationPtr &si):
        MotionValidator(si)
    {}
    ~R3SegmentValidator(){}

    virtual bool checkMotion(const SamplingVolume::StateType *s1, const SamplingVolume::StateType *s2) const
    {
        SamplingVolumeConstPtr regularizedGoal = boost::dynamic_pointer_cast<SamplingVolume const>(si_->getStateSpace());
        tPointSpec A, B;
        s1->get(A);
        s2->get(B);
        bool retq = true;
        //if((!regularizedGoal->inVolume(A)) || (!regularizedGoal->inVolume(B)))
        //    retq = false;
        if(retq)
            retq = !(regularizedGoal->intersectsSegment(A, B));
        return retq;
    }
    virtual bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const
    {
        SamplingVolume::StateType const*A = dynamic_cast<const SamplingVolume::StateType *>(s1);
        SamplingVolume::StateType const*B = dynamic_cast<const SamplingVolume::StateType *>(s2);
        bool retq = checkMotion(A, B);
        return retq;
    }
    virtual bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2, std::pair<ompl::base::State*, double> &lastValid) const
    {
        SamplingVolume::StateType const*A = dynamic_cast<const SamplingVolume::StateType *>(s1);
        SamplingVolume::StateType const*B = dynamic_cast<const SamplingVolume::StateType *>(s2);
        bool valid = checkMotion(A, B);
        // a bit of a cheat here, should actually check for the segment intersection.
        if(valid)
        {
            si_->copyState(lastValid.first, s2);
            lastValid.second = 1.0;
        }
        else
        {
            si_->copyState(lastValid.first, s1);
            lastValid.second = 0.0;
        }
        return valid;
    }
protected:
};



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

bool Maneuver::setFromSample(std::vector<tPointSpec> const& samples, meshproc_csg::MeshEntry const& forbidden)
{
    if(!shapeFromSample(samples))
        return false;
    return filterForbidden(forbidden);
}

bool Maneuver::setFromSample(std::vector<tPointSpec> const& samples, meshproc_csg::MeshEntry const& forbidden, SamplingVolumePtr const& regularizedGoal, SamplingVolumeConstPtr const& collector)
{
    if((!collector->isInited()) || (!regularizedGoal->isInited()))
        return setFromSample(samples, forbidden);

    tPointSpec direction;
    tPointSpec centroid;
    tPointSpec goal;

    getCentroid(samples, centroid);
    //Query whether centroid is inside the collector; if so, use the PCA as a direction.
    //centroid = tPointSpec(1, 0, 0);
    if(collector->inVolume(centroid))
    {
        meshproc_csg::MeshEntry vC(collector->getMeshEntry());
        vC.writeToFile("/home/blandc/Documents/collMM.stl");
        std::cout << "Controid in collector volume." << std::endl;
        std::cout << "Centroid:: " << ::CGAL::to_double(centroid.x()) << " " << ::CGAL::to_double(centroid.y()) << " " << ::CGAL::to_double(centroid.z()) << std::endl;
        Maneuver::getPCA(samples, direction);
        //direction = tPointSpec(0, 1, 0);
        std::cout << "Direction::" << ::CGAL::to_double(direction.x()) << " " << ::CGAL::to_double(direction.y()) << " " << ::CGAL::to_double(direction.z()) << std::endl;
    }
    else
    {
        bool haveDir = collector->closestPoint2Point(centroid, goal);
        if(!haveDir)
            return false;
        //centroid = tPointSpec(-0.5, -0.5, 0);
        //goal = tPointSpec(-0.3, 0, 0.01);
        std::cout << "Centroid:: " << ::CGAL::to_double(centroid.x()) << " " << ::CGAL::to_double(centroid.y()) << " " << ::CGAL::to_double(centroid.z()) << std::endl;
        std::cout << "Goal:: " << ::CGAL::to_double(goal.x()) << " " << ::CGAL::to_double(goal.y()) << " " << ::CGAL::to_double(goal.z()) << std::endl;
        bool fD = findDirection(centroid, goal, regularizedGoal, direction);
        if(!fD)
            return false;
    }

    if(!shapeFromSample(samples, direction))
        return false;
    return filterForbidden(forbidden, regularizedGoal, collector);
}

bool Maneuver::findDirection(tPointSpec const& centroid, tPointSpec const& goal, SamplingVolumePtr const& regularizedGoal, tPointSpec &direction) const
{
    ompl::base::ScopedState<SamplingVolume> startState(regularizedGoal);
    ompl::base::ScopedState<SamplingVolume> goalState(regularizedGoal);
    startState.get()->set(centroid);
    goalState.get()->set(goal);
    ompl::geometric::SimpleSetup ss(regularizedGoal);
    ompl::base::MotionValidatorPtr motionValidator(new R3SegmentValidator(ss.getSpaceInformation()));

    ss.getSpaceInformation()->setMotionValidator(motionValidator);
    ss.setStateValidityChecker(boost::bind(&trivialStateChecker, _1));
    ss.setStartAndGoalStates(startState, goalState);
    ompl::base::PlannerPtr planner(new ompl::geometric::RRTConnect(ss.getSpaceInformation()));
    ss.setPlanner(planner);

    ompl::base::PlannerStatus solved = ss.solve(1.0);

    if(!solved)
        return false;
    ss.simplifySolution(1.0);
    int maxK = ss.getSolutionPath().getStateCount();
    if(maxK < 2)
        return false;
    tPointSpec A, B;
    (static_cast<VolumeSample*>(ss.getSolutionPath().getState(0)))->get(A);
    (static_cast<VolumeSample*>(ss.getSolutionPath().getState(1)))->get(B);
    double x, y, z;
    x = ::CGAL::to_double(B.x()) - ::CGAL::to_double(A.x());
    y = ::CGAL::to_double(B.y()) - ::CGAL::to_double(A.y());
    z = ::CGAL::to_double(B.z()) - ::CGAL::to_double(A.z());
    double l = std::sqrt(x*x + y*y + z*z);
    x /= l;
    y /= l;
    z /= l;

    direction = tPointSpec(x, y, z);
    return true;
}

bool Maneuver::getMaxVolumeCC(meshproc_csg::MeshEntry & v) const
{
    std::map<meshproc_csg::Polyhedron::Facet_const_handle, std::size_t> fcm;

    int maxK = getCCs(volume->getMesh(), fcm);

    //std::cout << "GCC" << std::endl;

    std::vector<double> ccVolumes;
    ccVolumes.clear();
    ccVolumes.resize(maxK, 0.0);
    BOOST_FOREACH(meshproc_csg::Polyhedron::Facet_const_handle f, CGAL::faces(volume->getMesh()))
        ccVolumes[fcm[f]] += getSignedTriangleVolume(*f);

    //std::cout << "CCV" << std::endl;

    int argMax = 0;
    for(int k = 1; k < maxK; k++)
        if(ccVolumes[argMax] < ccVolumes[k])
            argMax = k;
    shape_msgs::Mesh meshMsg;
    meshMsg.triangles.clear();
    meshMsg.vertices.clear();

    //std::cout << "MCC" << std::endl;

    BOOST_FOREACH(meshproc_csg::Polyhedron::Facet_const_handle f, CGAL::faces(volume->getMesh()))
    {
            //std::cout << "Face CC index " << fcm[f] << " vs " << argMax << std::endl;
            if(fcm[f] == argMax)
            {
                meshproc_csg::Polyhedron::Halfedge_around_facet_const_circulator hc_a = f->facet_begin();
                meshproc_csg::Polyhedron::Halfedge_around_facet_const_circulator hc_b = hc_a; hc_b++;
                meshproc_csg::Polyhedron::Halfedge_around_facet_const_circulator hc_c = hc_b; hc_c++;

                double xA, yA, zA;
                double xB, yB, zB;
                double xC, yC, zC;

                xA = ::CGAL::to_double(hc_a->vertex()->point().x());
                yA = ::CGAL::to_double(hc_a->vertex()->point().y());
                zA = ::CGAL::to_double(hc_a->vertex()->point().z());

                xB = ::CGAL::to_double(hc_b->vertex()->point().x());
                yB = ::CGAL::to_double(hc_b->vertex()->point().y());
                zB = ::CGAL::to_double(hc_b->vertex()->point().z());

                xC = ::CGAL::to_double(hc_c->vertex()->point().x());
                yC = ::CGAL::to_double(hc_c->vertex()->point().y());
                zC = ::CGAL::to_double(hc_c->vertex()->point().z());
                geometry_msgs::Point p;
                int idx = meshMsg.vertices.size();
                p.x = xA; p.y = yA; p.z = zA;
                meshMsg.vertices.push_back(p);
                p.x = xB; p.y = yB; p.z = zB;
                meshMsg.vertices.push_back(p);
                p.x = xC; p.y = yC; p.z = zC;
                meshMsg.vertices.push_back(p);

                //std::cout << "V0 " << xA << " " << yA << " " << zA << std::endl;
                //std::cout << "V1 " << xB << " " << yB << " " << zB << std::endl;
                //std::cout << "V2 " << xC << " " << yC << " " << zC << std::endl;

                shape_msgs::MeshTriangle t;
                t.vertex_indices[0] = idx;
                t.vertex_indices[1] = idx + 1;
                t.vertex_indices[2] = idx + 2;
                meshMsg.triangles.push_back(t);
            }
    }

    v.loadFromMsg(meshMsg, 0.00001);

    return true;
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
    volume->getVolume();
    meshproc_csg::MeshEntry v;
    getMaxVolumeCC(v);
    SamplingVolume svol;
    svol.update(v);
    svol.getVertices(samples);
    bool haveShape = shapeFromSample(samples);
    if(!haveShape)
        return false;
    meshproc_csg::MeshEntry diff;
    diff.setFromIntersection(*volume, forbidden);
    if(0.00001 < diff.getVolume())
        return false;//filterForbidden(forbidden);
    return true;
}
bool Maneuver::filterForbidden(meshproc_csg::MeshEntry const& forbidden, SamplingVolumePtr const& regularizedGoal, SamplingVolumeConstPtr const& collector)
{
    if(!collector->isInited())
        return filterForbidden(forbidden);
    /*
     *  - substract forbidden from volume <- ok
     *  - select connected component of highest volume from the result <- check that we have sep. by connected comps
     *  - sample connected component to reinit g+, g-, v, ps; <- ok-ish, needs a def. of the other class
    */
    std::vector<tPointSpec> samples;
    tPointSpec direction, centroid, goal;
    if(!inited)
        return false;
    std::cout << "CSG_0" << std::endl;
    volume->setFromIntersection(*volume, regularizedGoal->getMeshEntry());
    std::cout << "CSG_1" << std::endl;
    volume->setFromDifference(*volume, forbidden);
    std::cout << "CSG_2" << std::endl;
    volume->setFromDifference(*volume, collector->getMeshEntry());
    std::cout << "CSG_3" << std::endl;
    volume->getVolume();
    std::cout << "CSG" << std::endl;
    meshproc_csg::MeshEntry v;
    meshproc_csg::MeshEntry vC0(*volume);
    vC0.writeToFile("/home/blandc/Documents/vVM0.stl");
    getMaxVolumeCC(v);
    std::cout << "GV" << std::endl;
    SamplingVolume svol;
    svol.update(v);
    svol.getVertices(samples);

    meshproc_csg::MeshEntry vC(*volume);
    vC.writeToFile("/home/blandc/Documents/vVM.stl");
    meshproc_csg::MeshEntry vC2(svol.getMeshEntry());
    vC2.writeToFile("/home/blandc/Documents/maxVM.stl");

    getCentroid(samples, centroid);
    if(!collector->closestPoint2Point(centroid, goal))
        return false;

    std::cout << "Centroid:: " << ::CGAL::to_double(centroid.x()) << " " << ::CGAL::to_double(centroid.y()) << " " << ::CGAL::to_double(centroid.z()) << std::endl;
    std::cout << "Goal:: " << ::CGAL::to_double(goal.x()) << " " << ::CGAL::to_double(goal.y()) << " " << ::CGAL::to_double(goal.z()) << std::endl;
    bool fD = findDirection(centroid, goal, regularizedGoal, direction);
    if(!fD)
    {
        std::cout << "Huh. Trying again." << std::endl;
        fD = findDirection(centroid, goal, regularizedGoal, direction);
    }
    if(!fD)
        return false;

    bool haveShape = shapeFromSample(samples, direction);
    if(!haveShape)
        return false;
    meshproc_csg::MeshEntry diff;
    diff.setFromIntersection(*volume, forbidden);
    double fVol = diff.getVolume();
    //diff.setFromIntersection(*volume, collector->getMeshEntry());
    //fVol += diff.getVolume();
    if(0.00001 < fVol)
        return false;//filterForbidden(forbidden, regularizedGoal, collector);
    return true;
}

MeshEntryConstPtr Maneuver::getGoalPlus(void) const
{
    if(goalPlus.get())
        return goalPlus;
    else
        return MeshEntryConstPtr(new meshproc_csg::MeshEntry());
}
MeshEntryConstPtr Maneuver::getGoalMinus(void) const
{
    if(goalMinus.get())
        return goalMinus;
    else
        return MeshEntryConstPtr(new meshproc_csg::MeshEntry());
}
std::vector<double> Maneuver::getParameters(void) const
{
    return parameters;
}

bool Maneuver::getCentroid(std::vector<tPointSpec> const& samples, tPointSpec& centroid)
{
    int maxK = samples.size();
    if(!maxK)
        return false;
    double x, y, z;
    x = y = z = 0.0;
    for(int k = 0; k < maxK; k++)
    {
        x += ::CGAL::to_double(samples[k].x());
        y += ::CGAL::to_double(samples[k].y());
        z += ::CGAL::to_double(samples[k].z());
    }
    x /= maxK;
    y /= maxK;
    z /= maxK;
    centroid = tPointSpec(x, y, z);
    return true;
}

bool Maneuver::getPCA(std::vector<tPointSpec> const& samples, tPointSpec& direction)
{
    if(!samples.size())
        return false;
    typedef CGAL::Simple_cartesian<double> SKernel;
    typedef SKernel::Point_3 Point;
    typedef SKernel::Line_3 Line;
    std::vector<Point> samplesS;
    int maxK = samples.size();
    samplesS.resize(maxK);
    for(int k = 0; k < maxK; k++)
        samplesS[k] = Point(::CGAL::to_double(samples[k].x()),
                            ::CGAL::to_double(samples[k].y()),
                            ::CGAL::to_double(samples[k].z()));
    Line line;
    CGAL::linear_least_squares_fitting_3(samplesS.begin(), samplesS.end(), line, CGAL::Dimension_tag<0>());
    Point A = line.point(0);
    Point B = line.point(1);
    double ax, ay, az, bx, by, bz;
    ax = ::CGAL::to_double(A.x());
    ay = ::CGAL::to_double(A.y());
    az = ::CGAL::to_double(A.z());
    bx = ::CGAL::to_double(B.x());
    by = ::CGAL::to_double(B.y());
    bz = ::CGAL::to_double(B.z());
    double dx, dy, dz;
    dx = bx - ax;
    dy = by - ay;
    dz = bz - az;
    double l = std::sqrt(dx*dx + dy*dy + dz*dz);
    if(l < 0.000001)
        return false;
    direction = tPointSpec(dx/l, dy/l, dz/l);
    return true;
}

bool Maneuver::getExtension(std::vector<tPointSpec> const& samples, tPointSpec const& centroid, tPointSpec const& direction, tPointSpec& A, tPointSpec& B)
{
    if(!samples.size())
        return false;
    double exMax = 0;
    double exMin = 0;
    int maxK = samples.size();
    double ox, oy, oz;
    ox = ::CGAL::to_double(centroid.x());
    oy = ::CGAL::to_double(centroid.y());
    oz = ::CGAL::to_double(centroid.z());
    double dx, dy, dz;
    dx = ::CGAL::to_double(direction.x());
    dy = ::CGAL::to_double(direction.y());
    dz = ::CGAL::to_double(direction.z());
    for(int k = 0; k < maxK; k++)
    {
        double px, py, pz;
        px = ::CGAL::to_double(samples[k].x());
        py = ::CGAL::to_double(samples[k].y());
        pz = ::CGAL::to_double(samples[k].z());
        double t = (px-ox)*dx + (py-oy)*dy + (pz-oz)*dz;
        if(t < exMin)
            exMin = t;
        if(t > exMax)
            exMax = t;
    }
    A = tPointSpec(ox + exMin*dx, oy + exMin*dy, oz + exMin*dz);
    B = tPointSpec(ox + exMax*dx, oy + exMax*dy, oz + exMax*dz);
    return true;
}

bool Maneuver::getAxes(tPointSpec const& directionX, tPointSpec const& refZ, tPointSpec &directionY, tPointSpec & directionZ)
{
    double xx, xy, xz;
    xx = ::CGAL::to_double(directionX.x());
    xy = ::CGAL::to_double(directionX.y());
    xz = ::CGAL::to_double(directionX.z());
    double rx, ry, rz;
    rx = ::CGAL::to_double(refZ.x());
    ry = ::CGAL::to_double(refZ.y());
    rz = ::CGAL::to_double(refZ.z());
    double l = rx*xx + ry*xy + rz*xz;
    if(l < 0.000001)
        directionZ = refZ;
    else
    {
        double zx, zy, zz;
        zx = rx - l*xx;
        zy = ry - l*xy;
        zz = rz - l*xz;
        l = std::sqrt(zx*zx + zy*zy + zz*zz);
        directionZ = tPointSpec(zx/l, zy/l, zz/l);
    }

    double zx, zy, zz;
    zx = ::CGAL::to_double(directionZ.x());
    zy = ::CGAL::to_double(directionZ.y());
    zz = ::CGAL::to_double(directionZ.z());
    directionY = tPointSpec(zy*xz - zz*xy, zz*xx - zx*xz, zx*xy - zy*xx);
    return true;
}

bool Maneuver::movePoint(tPointSpec const& P, tPointSpec const& direction, double t, tPointSpec & R)
{
    double px, py, pz;
    px = ::CGAL::to_double(P.x());
    py = ::CGAL::to_double(P.y());
    pz = ::CGAL::to_double(P.z());
    double dx, dy, dz;
    dx = ::CGAL::to_double(direction.x());
    dy = ::CGAL::to_double(direction.y());
    dz = ::CGAL::to_double(direction.z());
    R = tPointSpec(px + t*dx, py + t*dy, pz + t*dz);
    return true;
}

}
