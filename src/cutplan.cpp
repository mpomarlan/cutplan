#include <ros/ros.h>

#include <cutplan/ExampleManeuvers.h>
#include <cutplan/ExamplePlanners.h>

#include <meshproc_msgs/LoadMesh.h>
#include <meshproc_msgs/UnloadMesh.h>
#include <meshproc_msgs/GetLoadedMeshNames.h>
#include <meshproc_msgs/GetMesh.h>

#include <cutplan/GetManeuver.h>

typedef std::map<std::string, cutplan::SamplingVolumePtr> MeshMap;
MeshMap loadedMeshes;

bool do_LoadMesh(meshproc_msgs::LoadMesh::Request &req,
                 meshproc_msgs::LoadMesh::Response &res)
{
    ROS_INFO("Loading mesh %s", req.mesh_name.c_str());
    res.loaded_mesh = res.io_error = res.mesh_already_loaded = false;
    MeshMap::iterator it = loadedMeshes.find(req.mesh_name);
    if(it != loadedMeshes.end())
        res.mesh_already_loaded = true;
    else
    {
        if((0 == req.mesh_filenames.size()) && (0 == req.mesh_msgs.size()))
            return true;
        cutplan::SamplingVolumePtr newSVol(new cutplan::SamplingVolume());
        it = loadedMeshes.insert(loadedMeshes.begin(),
                                 std::pair<std::string, cutplan::SamplingVolumePtr>(req.mesh_name, newSVol));
        meshproc_csg::MeshEntry R;
        int maxK = req.mesh_filenames.size();
        bool goOn = true;
        for(int k = 0; (k < maxK) && goOn; k++)
        {
            ROS_INFO("    loading part from file %s", req.mesh_filenames[k].c_str());
            goOn = R.loadFromFile(req.mesh_filenames[k], req.duplicate_dist);
        }
        if(!goOn)
        {
            ROS_INFO("    Encountered I/O error (file might not exist or is inaccessible), cancelling load.");
            res.io_error = true;
            loadedMeshes.erase(it);
            return true;
        }
        res.loaded_mesh = true;
        maxK = req.mesh_msgs.size();
        for(int k = 0; k < maxK; k++)
        {
            ROS_INFO("    loading part from message");
            R.loadFromMsg(req.mesh_msgs[k], req.duplicate_dist);
        }
        newSVol->update(R);
    }
    ROS_INFO("    Loading done.");
    return true;
}

bool do_UnloadMesh(meshproc_msgs::UnloadMesh::Request &req,
                   meshproc_msgs::UnloadMesh::Response &res)
{
    res.unloaded_mesh = 0;
    MeshMap::iterator it = loadedMeshes.find(req.mesh_name);
    if(loadedMeshes.end() != it)
    {
        res.unloaded_mesh = 1;
        loadedMeshes.erase(it);
    }
    return true;
}

bool do_GetLoadedMeshNames(meshproc_msgs::GetLoadedMeshNames::Request &req,
                           meshproc_msgs::GetLoadedMeshNames::Response &res)
{
    res.mesh_names.clear();
    MeshMap::iterator it = loadedMeshes.begin();
    for(it = loadedMeshes.begin(); it != loadedMeshes.end(); it++)
    {
        res.mesh_names.push_back(it->first);
    }
    return true;
}

bool do_GetMesh(meshproc_msgs::GetMesh::Request &req,
                meshproc_msgs::GetMesh::Response &res)
{
    MeshMap::iterator itA = loadedMeshes.find(req.mesh_name);
    res.mesh_loaded = true;
    res.file_written = false;
    if(itA == loadedMeshes.end())
    {
        res.mesh_loaded = false;
        return true;
    }
    if(req.return_result)
    {
        meshproc_csg::MeshEntry cp(itA->second->getMeshEntry());
        cp.writeToMsg(res.result);
    }
    else
        res.result = shape_msgs::Mesh();
    if(req.result_to_file)
    {
        meshproc_csg::MeshEntry cp(itA->second->getMeshEntry());
        res.file_written = cp.writeToFile(req.result_filename);
    }
    return true;
}

bool do_GetManeuver(cutplan::GetManeuver::Request &req,
                    cutplan::GetManeuver::Response &res)
{
    MeshMap::iterator collIt = loadedMeshes.end();
    MeshMap::iterator goalIt = loadedMeshes.end();
    MeshMap::iterator forbIt = loadedMeshes.end();
    MeshMap::iterator gPlsIt = loadedMeshes.end();
    MeshMap::iterator gMnsIt = loadedMeshes.end();
    MeshMap::iterator nwGlIt = loadedMeshes.end();

    bool useForbidden = false;
    bool useCollector = false;

    res.mesh_goal_loaded = res.mesh_collector_loaded = res.mesh_forbidden_loaded = false;

    //TODO: Need to make this cleaner.
    res.have_maneuver = false;
    if((req.maneuver == "burr") ||
       (req.maneuver == "mid_wiper") ||
       (req.maneuver == "brush"))
        res.have_maneuver = true;

    goalIt = loadedMeshes.find(req.goal);
    if(goalIt != loadedMeshes.end())
        res.mesh_goal_loaded = true;

    if(req.collector != "")
    {
        collIt = loadedMeshes.find(req.collector);
        useCollector = true;
        if(collIt != loadedMeshes.end())
            res.mesh_collector_loaded = true;
    }

    if(req.forbidden != "")
    {
        collIt = loadedMeshes.find(req.forbidden);
        useForbidden = true;
        if(collIt != loadedMeshes.end())
            res.mesh_forbidden_loaded = true;
    }

    if((!res.have_maneuver) || (!res.mesh_goal_loaded)
            || (useForbidden && (!res.mesh_forbidden_loaded))
            || (useCollector && (!res.mesh_collector_loaded)))
    {
        res.operation_done = false;
        return true;
    }

    cutplan::SamplingVolumePtr fVol;
    cutplan::SamplingVolumePtr cVol;
    cutplan::SamplingVolumePtr rVol;
    cutplan::SamplingVolumePtr gPlus(new cutplan::SamplingVolume());
    cutplan::SamplingVolumePtr gMinus(new cutplan::SamplingVolume());
    cutplan::SamplingVolumePtr gNew(new cutplan::SamplingVolume());

    if(!useForbidden)
        fVol.reset(new cutplan::SamplingVolume());
    else
        fVol = forbIt->second;
    if(!useCollector)
        cVol.reset(new cutplan::SamplingVolume());
    else
        cVol = collIt->second;

    meshproc_csg::MeshEntry regGoalMesh(goalIt->second->getMeshEntry());
    regGoalMesh.setFromUnion(regGoalMesh, cVol->getMeshEntry());
    regGoalMesh.setFromConvexHull(regGoalMesh);
    rVol.reset(new cutplan::SamplingVolume());
    rVol->update(regGoalMesh);

    cutplan::ManeuverPtr maneuver;
    std::vector<cutplan::tPointSpec> samples;
    if(req.maneuver == "brush")
    {
        goalIt->second->getVertexSample(4, samples);
        maneuver.reset(new cutplan::Brush());
    }
    if(req.maneuver == "burr")
    {
        goalIt->second->getVertexSample(1, samples);
        maneuver.reset(new cutplan::Burr());
    }
    if(req.maneuver == "mid_wiper")
    {
        goalIt->second->getVertexSample(4, samples);
        maneuver.reset(new cutplan::Wiper());
    }
    if(useCollector)
        res.operation_done = maneuver->setFromSample(samples, fVol->getMeshEntry(), rVol, cVol);
    else
        res.operation_done = maneuver->setFromSample(samples, fVol->getMeshEntry());
    if(res.operation_done)
    {
        gPlus->update(maneuver->getGoalPlus());
        gMinus->update(maneuver->getGoalMinus());
        gNew->update(goalIt->second->getMeshEntry(), maneuver->getGoalPlus(), maneuver->getGoalMinus());
    }

    if(res.operation_done)
    {
        if(req.goal_plus != "")
        {
            gPlsIt = loadedMeshes.find(req.goal_plus);
            if(gPlsIt == loadedMeshes.end())
                gPlsIt = loadedMeshes.insert(loadedMeshes.begin(),
                                             std::pair<std::string, cutplan::SamplingVolumePtr>(req.goal_plus, cutplan::SamplingVolumePtr(new cutplan::SamplingVolume())));
        }
        if(req.goal_minus != "")
        {
            gPlsIt = loadedMeshes.find(req.goal_minus);
            if(gMnsIt == loadedMeshes.end())
                gMnsIt = loadedMeshes.insert(loadedMeshes.begin(),
                                             std::pair<std::string, cutplan::SamplingVolumePtr>(req.goal_minus, cutplan::SamplingVolumePtr(new cutplan::SamplingVolume())));
        }
        if(req.new_goal != "")
        {
            nwGlIt = loadedMeshes.find(req.new_goal);
            if(nwGlIt == loadedMeshes.end())
                nwGlIt = loadedMeshes.insert(loadedMeshes.begin(),
                                             std::pair<std::string, cutplan::SamplingVolumePtr>(req.new_goal, cutplan::SamplingVolumePtr(new cutplan::SamplingVolume())));
        }

        if(nwGlIt != loadedMeshes.end())
            nwGlIt->second->update(gNew->getMeshEntry());
        if(gPlsIt != loadedMeshes.end())
            gPlsIt->second->update(gPlus->getMeshEntry());
        if(gMnsIt != loadedMeshes.end())
            gMnsIt->second->update(gMinus->getMeshEntry());
    }

    return true;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "cutplan");
    ros::NodeHandle n;

#if 0
    cutplan::Burr burr;
    meshproc_csg::MeshEntry forbidden;
    cutplan::tPointSpec p(0, 0, 0);
    std::vector<cutplan::tPointSpec> samples;
    samples.clear();
    samples.push_back(p);
    forbidden.loadFromFile("/home/blandc/Documents/forbidden.stl", 0.00001);
    ROS_INFO("SetFromSample ...");
    burr.setFromSample(samples, forbidden);
    ROS_INFO("              ... done.");
    std::vector<double> params;
    params = burr.getParameters();
    std::cout << " Params " << params[0] << " " << params[1] << " " << params[2] << std::endl;
    ROS_INFO("GetGoalMinus  ...");
    meshproc_csg::MeshEntry mesh(*(burr.getGoalMinus()));
    //mesh.setFromMeshEntry();
    ROS_INFO("              ... done.");
    mesh.writeToFile("/home/blandc/Documents/werd.stl");

    cutplan::SamplingVolume svol;
    svol.update(mesh);
    std::vector<cutplan::tPointSpec> aux;
    ROS_INFO("GetVertexSample  ...");
    bool res = svol.getVertexSample(3, aux);\
    std::cout<< "                 result " << res << std::endl;
    ROS_INFO("                 ... done.");
    std::cout << "    Sample 0: " << ::CGAL::to_double(aux[0].x()) << " " << ::CGAL::to_double(aux[0].y()) << " " << ::CGAL::to_double(aux[0].z()) << std::endl;
    std::cout << "    Sample 1: " << ::CGAL::to_double(aux[1].x()) << " " << ::CGAL::to_double(aux[1].y()) << " " << ::CGAL::to_double(aux[1].z()) << std::endl;
    std::cout << "    Sample 2: " << ::CGAL::to_double(aux[2].x()) << " " << ::CGAL::to_double(aux[2].y()) << " " << ::CGAL::to_double(aux[2].z()) << std::endl;

    svol.getVertices(aux);
    int maxK = aux.size();
    for(int k = 0; k < maxK; k++)
        std::cout << "    Vertex " << k << ": " << ::CGAL::to_double(aux[k].x()) << " " << ::CGAL::to_double(aux[k].y()) << " " << ::CGAL::to_double(aux[k].z()) << std::endl;

    // Try generating a wiper maneuver (give sample and direction).

    // Next, generate a wiper maneuver from a sample, collector, and regularized regions.

    cutplan::Wiper wiper;
    cutplan::SamplingVolumePtr wGoal(new cutplan::SamplingVolume()), regGoal(new cutplan::SamplingVolume()), collector(new cutplan::SamplingVolume());
    meshproc_csg::MeshEntry wGM, collM, rGM;
    collM.loadFromFile("/home/blandc/Documents/collM.stl", 0.00001);
    rGM.loadFromFile("/home/blandc/Documents/rGM.stl", 0.00001);
    wGM.loadFromFile("/home/blandc/Documents/wGM.stl", 0.00001);
    regGoal->update(rGM);
    collector->update(collM);
    wGoal->update(wGM);
    samples.clear();
    wGoal->getVertexSample(4, samples);
    wiper.setFromSample(samples, forbidden, regGoal, collector);
    meshproc_csg::MeshEntry wiperMeshMinus(*(wiper.getGoalMinus()));
    meshproc_csg::MeshEntry wiperMeshPlus(*(wiper.getGoalPlus()));
    wiperMeshMinus.writeToFile("/home/blandc/Documents/werdWiperMinus.stl");
    wiperMeshPlus.writeToFile("/home/blandc/Documents/werdWiperPlus.stl");
    std::vector<double> wiperParams = wiper.getParameters();
    std::cerr << "Wiper Params" << wiperParams[0] << " " << wiperParams[1] << " " << wiperParams[2] << " " << wiperParams[3] << " " << wiperParams[4] << " " << wiperParams[5] << std::endl;
#endif


    ROS_INFO("              ... done.");
    ROS_INFO("Advertising services ...");
    ros::ServiceServer LoadMesh_service = n.advertiseService("cutplan/LoadMesh", do_LoadMesh);
    ros::ServiceServer UnloadMesh_service = n.advertiseService("cutplan/UnloadMesh", do_UnloadMesh);
    ros::ServiceServer GetLoadedMeshNames_service = n.advertiseService("cutplan/GetLoadedMeshNames", do_GetLoadedMeshNames);
    ros::ServiceServer GetMesh_service = n.advertiseService("cutplan/GetMesh", do_GetMesh);
    ros::ServiceServer GetManeuver_service = n.advertiseService("cutplan/GetManeuver", do_GetManeuver);
    ROS_INFO(" ... all done.");

    ros::spin();
    return 0;
}

