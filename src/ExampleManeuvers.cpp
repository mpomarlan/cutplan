#include <cutplan/ExampleManeuvers.h>
#include <shape_msgs/Mesh.h>

namespace cutplan
{

bool Brush::shapeFromSample(std::vector<tPointSpec> const& samples)
{
    if(!samples.size())
        return false;

    tPointSpec centroid, direction, A, B;
    Maneuver::getCentroid(samples, centroid);
    Maneuver::getPCA(samples, direction);
    Maneuver::getExtension(samples, centroid, direction, A, B);
    tPointSpec dY, dZ;
    Maneuver::getAxes(direction, tPointSpec(0, 0, 1), dY, dZ);
    tPointSpec sA, eB;
    Maneuver::movePoint(A, direction, 0.002, sA);
    Maneuver::movePoint(B, direction, -0.002, eB);
    tPointSpec cAHH, cAHL, cALL, cALH;
    Maneuver::movePoint(A, dY, 0.002, cAHH); Maneuver::movePoint(cAHH, dZ, 0.05, cAHH);
    Maneuver::movePoint(A, dY, 0.002, cAHL); Maneuver::movePoint(cAHL, dZ, -0.05, cAHL);
    Maneuver::movePoint(A, dY, -0.002, cALL); Maneuver::movePoint(cALL, dZ, -0.05, cALL);
    Maneuver::movePoint(A, dY, -0.002, cALH); Maneuver::movePoint(cALH, dZ, 0.05, cALH);
    tPointSpec cBHH, cBHL, cBLL, cBLH;
    Maneuver::movePoint(B, dY, 0.002, cBHH); Maneuver::movePoint(cBHH, dZ, 0.05, cBHH);
    Maneuver::movePoint(B, dY, 0.002, cBHL); Maneuver::movePoint(cBHL, dZ, -0.05, cBHL);
    Maneuver::movePoint(B, dY, -0.002, cBLL); Maneuver::movePoint(cBLL, dZ, -0.05, cBLL);
    Maneuver::movePoint(B, dY, -0.002, cBLH); Maneuver::movePoint(cBLH, dZ, 0.05, cBLH);

    parameters.resize(6);
    parameters[0] = ::CGAL::to_double(A.x());
    parameters[1] = ::CGAL::to_double(A.y());
    parameters[2] = ::CGAL::to_double(A.z());
    parameters[3] = ::CGAL::to_double(B.x());
    parameters[4] = ::CGAL::to_double(B.y());
    parameters[5] = ::CGAL::to_double(B.z());
    volume.reset(new meshproc_csg::MeshEntry());
    shape_msgs::Mesh msg;
    msg.vertices.resize(8);
    msg.triangles.resize(12);
    msg.vertices[0].x = ::CGAL::to_double(cAHH.x());
    msg.vertices[0].y = ::CGAL::to_double(cAHH.y());
    msg.vertices[0].z = ::CGAL::to_double(cAHH.z());
    msg.vertices[1].x = ::CGAL::to_double(cAHL.x());
    msg.vertices[1].y = ::CGAL::to_double(cAHL.y());
    msg.vertices[1].z = ::CGAL::to_double(cAHL.z());
    msg.vertices[2].x = ::CGAL::to_double(cALL.x());
    msg.vertices[2].y = ::CGAL::to_double(cALL.y());
    msg.vertices[2].z = ::CGAL::to_double(cALL.z());
    msg.vertices[3].x = ::CGAL::to_double(cALH.x());
    msg.vertices[3].y = ::CGAL::to_double(cALH.y());
    msg.vertices[3].z = ::CGAL::to_double(cALH.z());
    msg.vertices[4].x = ::CGAL::to_double(cBHH.x());
    msg.vertices[4].y = ::CGAL::to_double(cBHH.y());
    msg.vertices[4].z = ::CGAL::to_double(cBHH.z());
    msg.vertices[5].x = ::CGAL::to_double(cBHL.x());
    msg.vertices[5].y = ::CGAL::to_double(cBHL.y());
    msg.vertices[5].z = ::CGAL::to_double(cBHL.z());
    msg.vertices[6].x = ::CGAL::to_double(cBLL.x());
    msg.vertices[6].y = ::CGAL::to_double(cBLL.y());
    msg.vertices[6].z = ::CGAL::to_double(cBLL.z());
    msg.vertices[7].x = ::CGAL::to_double(cBLH.x());
    msg.vertices[7].y = ::CGAL::to_double(cBLH.y());
    msg.vertices[7].z = ::CGAL::to_double(cBLH.z());
    msg.triangles[0].vertex_indices[0] = 0;
    msg.triangles[0].vertex_indices[1] = 2;
    msg.triangles[0].vertex_indices[2] = 1;
    msg.triangles[1].vertex_indices[0] = 0;
    msg.triangles[1].vertex_indices[1] = 3;
    msg.triangles[1].vertex_indices[2] = 2;
    msg.triangles[2].vertex_indices[0] = 0;
    msg.triangles[2].vertex_indices[1] = 1;
    msg.triangles[2].vertex_indices[2] = 5;
    msg.triangles[3].vertex_indices[0] = 1;
    msg.triangles[3].vertex_indices[1] = 2;
    msg.triangles[3].vertex_indices[2] = 6;
    msg.triangles[4].vertex_indices[0] = 2;
    msg.triangles[4].vertex_indices[1] = 3;
    msg.triangles[4].vertex_indices[2] = 7;
    msg.triangles[5].vertex_indices[0] = 3;
    msg.triangles[5].vertex_indices[1] = 0;
    msg.triangles[5].vertex_indices[2] = 4;
    msg.triangles[6].vertex_indices[0] = 0;
    msg.triangles[6].vertex_indices[1] = 5;
    msg.triangles[6].vertex_indices[2] = 4;
    msg.triangles[7].vertex_indices[0] = 1;
    msg.triangles[7].vertex_indices[1] = 6;
    msg.triangles[7].vertex_indices[2] = 5;
    msg.triangles[8].vertex_indices[0] = 2;
    msg.triangles[8].vertex_indices[1] = 7;
    msg.triangles[8].vertex_indices[2] = 6;
    msg.triangles[9].vertex_indices[0] = 3;
    msg.triangles[9].vertex_indices[1] = 4;
    msg.triangles[9].vertex_indices[2] = 7;
    msg.triangles[10].vertex_indices[0] = 4;
    msg.triangles[10].vertex_indices[1] = 5;
    msg.triangles[10].vertex_indices[2] = 6;
    msg.triangles[11].vertex_indices[0] = 4;
    msg.triangles[11].vertex_indices[1] = 6;
    msg.triangles[11].vertex_indices[2] = 7;
    volume->loadFromMsg(msg, 0.00001);
    goalMinus = volume;
    goalPlus.reset();
    inited = true;
    return true;
}
bool Brush::shapeFromSample(std::vector<tPointSpec> const& samples, tPointSpec const& direction)
{
    return shapeFromSample(samples);
}
bool Burr::shapeFromSample(std::vector<tPointSpec> const& samples)
{
    if(!samples.size())
        return false;

    tPointSpec P = SamplingVolume::PSpecCentroid(samples);
    parameters.resize(3);
    parameters[0] = ::CGAL::to_double(P.x());
    parameters[1] = ::CGAL::to_double(P.y());
    parameters[2] = ::CGAL::to_double(P.z());
    volume.reset(new meshproc_csg::MeshEntry());
    shape_msgs::Mesh msg;
    msg.vertices.resize(8);
    msg.triangles.resize(12);
    msg.vertices[0].x = parameters[0] - 0.001;
    msg.vertices[0].y = parameters[1] - 0.001;
    msg.vertices[0].z = parameters[2] - 0.001;
    msg.vertices[1].x = parameters[0] + 0.001;
    msg.vertices[1].y = parameters[1] - 0.001;
    msg.vertices[1].z = parameters[2] - 0.001;
    msg.vertices[2].x = parameters[0] + 0.001;
    msg.vertices[2].y = parameters[1] + 0.001;
    msg.vertices[2].z = parameters[2] - 0.001;
    msg.vertices[3].x = parameters[0] - 0.001;
    msg.vertices[3].y = parameters[1] + 0.001;
    msg.vertices[3].z = parameters[2] - 0.001;
    msg.vertices[4].x = parameters[0] - 0.001;
    msg.vertices[4].y = parameters[1] - 0.001;
    msg.vertices[4].z = parameters[2] + 0.001;
    msg.vertices[5].x = parameters[0] + 0.001;
    msg.vertices[5].y = parameters[1] - 0.001;
    msg.vertices[5].z = parameters[2] + 0.001;
    msg.vertices[6].x = parameters[0] + 0.001;
    msg.vertices[6].y = parameters[1] + 0.001;
    msg.vertices[6].z = parameters[2] + 0.001;
    msg.vertices[7].x = parameters[0] - 0.001;
    msg.vertices[7].y = parameters[1] + 0.001;
    msg.vertices[7].z = parameters[2] + 0.001;
    msg.triangles[0].vertex_indices[0] = 0;
    msg.triangles[0].vertex_indices[1] = 2;
    msg.triangles[0].vertex_indices[2] = 1;
    msg.triangles[1].vertex_indices[0] = 0;
    msg.triangles[1].vertex_indices[1] = 3;
    msg.triangles[1].vertex_indices[2] = 2;
    msg.triangles[2].vertex_indices[0] = 0;
    msg.triangles[2].vertex_indices[1] = 1;
    msg.triangles[2].vertex_indices[2] = 5;
    msg.triangles[3].vertex_indices[0] = 1;
    msg.triangles[3].vertex_indices[1] = 2;
    msg.triangles[3].vertex_indices[2] = 6;
    msg.triangles[4].vertex_indices[0] = 2;
    msg.triangles[4].vertex_indices[1] = 3;
    msg.triangles[4].vertex_indices[2] = 7;
    msg.triangles[5].vertex_indices[0] = 3;
    msg.triangles[5].vertex_indices[1] = 0;
    msg.triangles[5].vertex_indices[2] = 4;
    msg.triangles[6].vertex_indices[0] = 0;
    msg.triangles[6].vertex_indices[1] = 5;
    msg.triangles[6].vertex_indices[2] = 4;
    msg.triangles[7].vertex_indices[0] = 1;
    msg.triangles[7].vertex_indices[1] = 6;
    msg.triangles[7].vertex_indices[2] = 5;
    msg.triangles[8].vertex_indices[0] = 2;
    msg.triangles[8].vertex_indices[1] = 7;
    msg.triangles[8].vertex_indices[2] = 6;
    msg.triangles[9].vertex_indices[0] = 3;
    msg.triangles[9].vertex_indices[1] = 4;
    msg.triangles[9].vertex_indices[2] = 7;
    msg.triangles[10].vertex_indices[0] = 4;
    msg.triangles[10].vertex_indices[1] = 5;
    msg.triangles[10].vertex_indices[2] = 6;
    msg.triangles[11].vertex_indices[0] = 4;
    msg.triangles[11].vertex_indices[1] = 6;
    msg.triangles[11].vertex_indices[2] = 7;
    volume->loadFromMsg(msg, 0.00001);
    goalMinus = volume;
    goalPlus.reset();
    inited = true;
    return true;
}
bool Burr::shapeFromSample(std::vector<tPointSpec> const& samples, tPointSpec const& direction)
{
    return shapeFromSample(samples);
}
bool Wiper::shapeFromSample(std::vector<tPointSpec> const& samples)
{
    return false;
}
bool Wiper::shapeFromSample(std::vector<tPointSpec> const& samples, tPointSpec const& direction)
{
    if(!samples.size())
        return false;

    tPointSpec centroid, A, B;
    Maneuver::getCentroid(samples, centroid);
    Maneuver::getExtension(samples, centroid, direction, A, B);

    std::cout << "Sample Centroid:: " << ::CGAL::to_double(centroid.x()) << " " << ::CGAL::to_double(centroid.y()) << " " << ::CGAL::to_double(centroid.z()) << std::endl;
    for(int k = 0; k < samples.size(); k++)
    {
        std::cout << "\tSample " << k << ": " << ::CGAL::to_double(samples[k].x()) << " "  << ::CGAL::to_double(samples[k].y()) << " "  << ::CGAL::to_double(samples[k].z()) << std::endl;
    }

    tPointSpec dY, dZ;
    Maneuver::getAxes(direction, tPointSpec(0, 0, 1), dY, dZ);
    tPointSpec sA, eB;
    Maneuver::movePoint(A, direction, 0.05, sA);
    Maneuver::movePoint(B, direction, -0.05, eB);
    tPointSpec cAHH, cAHL, cALL, cALH;
    std::cerr << "dZ " << dZ.x() << " " << dZ.y() << " " << dZ.z() << std::endl;
    std::cerr << "dY " << dY.x() << " " << dY.y() << " " << dY.z() << std::endl;
    std::cerr << "dX " << direction.x() << " " << direction.y() << " " << direction.z() << std::endl;
    Maneuver::movePoint(A, dY, 0.05, cAHH); Maneuver::movePoint(cAHH, dZ, 0.05, cAHH);
    Maneuver::movePoint(A, dY, 0.05, cAHL); Maneuver::movePoint(cAHL, dZ, -0.05, cAHL);
    Maneuver::movePoint(A, dY, -0.05, cALL); Maneuver::movePoint(cALL, dZ, -0.05, cALL);
    Maneuver::movePoint(A, dY, -0.05, cALH); Maneuver::movePoint(cALH, dZ, 0.05, cALH);
    tPointSpec cBHH, cBHL, cBLL, cBLH;
    Maneuver::movePoint(B, dY, 0.05, cBHH); Maneuver::movePoint(cBHH, dZ, 0.05, cBHH);
    Maneuver::movePoint(B, dY, 0.05, cBHL); Maneuver::movePoint(cBHL, dZ, -0.05, cBHL);
    Maneuver::movePoint(B, dY, -0.05, cBLL); Maneuver::movePoint(cBLL, dZ, -0.05, cBLL);
    Maneuver::movePoint(B, dY, -0.05, cBLH); Maneuver::movePoint(cBLH, dZ, 0.05, cBLH);

    parameters.resize(6);
    parameters[0] = ::CGAL::to_double(sA.x());
    parameters[1] = ::CGAL::to_double(sA.y());
    parameters[2] = ::CGAL::to_double(sA.z());
    parameters[3] = ::CGAL::to_double(eB.x());
    parameters[4] = ::CGAL::to_double(eB.y());
    parameters[5] = ::CGAL::to_double(eB.z());
    volume.reset(new meshproc_csg::MeshEntry());
    shape_msgs::Mesh msg;
    msg.vertices.resize(8);
    msg.triangles.resize(12);
    msg.vertices[0].x = ::CGAL::to_double(cAHH.x());
    msg.vertices[0].y = ::CGAL::to_double(cAHH.y());
    msg.vertices[0].z = ::CGAL::to_double(cAHH.z());
    msg.vertices[1].x = ::CGAL::to_double(cAHL.x());
    msg.vertices[1].y = ::CGAL::to_double(cAHL.y());
    msg.vertices[1].z = ::CGAL::to_double(cAHL.z());
    msg.vertices[2].x = ::CGAL::to_double(cALL.x());
    msg.vertices[2].y = ::CGAL::to_double(cALL.y());
    msg.vertices[2].z = ::CGAL::to_double(cALL.z());
    msg.vertices[3].x = ::CGAL::to_double(cALH.x());
    msg.vertices[3].y = ::CGAL::to_double(cALH.y());
    msg.vertices[3].z = ::CGAL::to_double(cALH.z());
    msg.vertices[4].x = ::CGAL::to_double(cBHH.x());
    msg.vertices[4].y = ::CGAL::to_double(cBHH.y());
    msg.vertices[4].z = ::CGAL::to_double(cBHH.z());
    msg.vertices[5].x = ::CGAL::to_double(cBHL.x());
    msg.vertices[5].y = ::CGAL::to_double(cBHL.y());
    msg.vertices[5].z = ::CGAL::to_double(cBHL.z());
    msg.vertices[6].x = ::CGAL::to_double(cBLL.x());
    msg.vertices[6].y = ::CGAL::to_double(cBLL.y());
    msg.vertices[6].z = ::CGAL::to_double(cBLL.z());
    msg.vertices[7].x = ::CGAL::to_double(cBLH.x());
    msg.vertices[7].y = ::CGAL::to_double(cBLH.y());
    msg.vertices[7].z = ::CGAL::to_double(cBLH.z());
    msg.triangles[0].vertex_indices[0] = 0;
    msg.triangles[0].vertex_indices[1] = 2;
    msg.triangles[0].vertex_indices[2] = 1;
    msg.triangles[1].vertex_indices[0] = 0;
    msg.triangles[1].vertex_indices[1] = 3;
    msg.triangles[1].vertex_indices[2] = 2;
    msg.triangles[2].vertex_indices[0] = 0;
    msg.triangles[2].vertex_indices[1] = 1;
    msg.triangles[2].vertex_indices[2] = 5;
    msg.triangles[3].vertex_indices[0] = 1;
    msg.triangles[3].vertex_indices[1] = 2;
    msg.triangles[3].vertex_indices[2] = 6;
    msg.triangles[4].vertex_indices[0] = 2;
    msg.triangles[4].vertex_indices[1] = 3;
    msg.triangles[4].vertex_indices[2] = 7;
    msg.triangles[5].vertex_indices[0] = 3;
    msg.triangles[5].vertex_indices[1] = 0;
    msg.triangles[5].vertex_indices[2] = 4;
    msg.triangles[6].vertex_indices[0] = 0;
    msg.triangles[6].vertex_indices[1] = 5;
    msg.triangles[6].vertex_indices[2] = 4;
    msg.triangles[7].vertex_indices[0] = 1;
    msg.triangles[7].vertex_indices[1] = 6;
    msg.triangles[7].vertex_indices[2] = 5;
    msg.triangles[8].vertex_indices[0] = 2;
    msg.triangles[8].vertex_indices[1] = 7;
    msg.triangles[8].vertex_indices[2] = 6;
    msg.triangles[9].vertex_indices[0] = 3;
    msg.triangles[9].vertex_indices[1] = 4;
    msg.triangles[9].vertex_indices[2] = 7;
    msg.triangles[10].vertex_indices[0] = 4;
    msg.triangles[10].vertex_indices[1] = 5;
    msg.triangles[10].vertex_indices[2] = 6;
    msg.triangles[11].vertex_indices[0] = 4;
    msg.triangles[11].vertex_indices[1] = 6;
    msg.triangles[11].vertex_indices[2] = 7;
    volume->loadFromMsg(msg, 0.00001);
    goalMinus = volume;

    std::cout << "LG-" << std::endl;

    goalPlus.reset(new meshproc_csg::MeshEntry());
    A = B;
    Maneuver::movePoint(A, direction, 0.005, B);
    Maneuver::movePoint(A, dY, 0.05, cAHH); Maneuver::movePoint(cAHH, dZ, 0.01, cAHH);
    Maneuver::movePoint(A, dY, 0.05, cAHL); Maneuver::movePoint(cAHL, dZ, -0.01, cAHL);
    Maneuver::movePoint(A, dY, -0.05, cALL); Maneuver::movePoint(cALL, dZ, -0.01, cALL);
    Maneuver::movePoint(A, dY, -0.05, cALH); Maneuver::movePoint(cALH, dZ, 0.01, cALH);
    Maneuver::movePoint(B, dY, 0.05, cBHH); Maneuver::movePoint(cBHH, dZ, 0.01, cBHH);
    Maneuver::movePoint(B, dY, 0.05, cBHL); Maneuver::movePoint(cBHL, dZ, -0.01, cBHL);
    Maneuver::movePoint(B, dY, -0.05, cBLL); Maneuver::movePoint(cBLL, dZ, -0.01, cBLL);
    Maneuver::movePoint(B, dY, -0.05, cBLH); Maneuver::movePoint(cBLH, dZ, 0.01, cBLH);
    msg.vertices[0].x = ::CGAL::to_double(cAHH.x());
    msg.vertices[0].y = ::CGAL::to_double(cAHH.y());
    msg.vertices[0].z = ::CGAL::to_double(cAHH.z());
    msg.vertices[1].x = ::CGAL::to_double(cAHL.x());
    msg.vertices[1].y = ::CGAL::to_double(cAHL.y());
    msg.vertices[1].z = ::CGAL::to_double(cAHL.z());
    msg.vertices[2].x = ::CGAL::to_double(cALL.x());
    msg.vertices[2].y = ::CGAL::to_double(cALL.y());
    msg.vertices[2].z = ::CGAL::to_double(cALL.z());
    msg.vertices[3].x = ::CGAL::to_double(cALH.x());
    msg.vertices[3].y = ::CGAL::to_double(cALH.y());
    msg.vertices[3].z = ::CGAL::to_double(cALH.z());
    msg.vertices[4].x = ::CGAL::to_double(cBHH.x());
    msg.vertices[4].y = ::CGAL::to_double(cBHH.y());
    msg.vertices[4].z = ::CGAL::to_double(cBHH.z());
    msg.vertices[5].x = ::CGAL::to_double(cBHL.x());
    msg.vertices[5].y = ::CGAL::to_double(cBHL.y());
    msg.vertices[5].z = ::CGAL::to_double(cBHL.z());
    msg.vertices[6].x = ::CGAL::to_double(cBLL.x());
    msg.vertices[6].y = ::CGAL::to_double(cBLL.y());
    msg.vertices[6].z = ::CGAL::to_double(cBLL.z());
    msg.vertices[7].x = ::CGAL::to_double(cBLH.x());
    msg.vertices[7].y = ::CGAL::to_double(cBLH.y());
    msg.vertices[7].z = ::CGAL::to_double(cBLH.z());
    goalPlus->loadFromMsg(msg, 0.00001);

    std::cout << "LG+" << std::endl;

    inited = true;
    return true;
}

}
