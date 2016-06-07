#ifndef __CUTPLAN_TYPEDEFS_H__

#define __CUTPLAN_TYPEDEFS_H__

#include <cstddef>
#include <string>
#include <vector>
#include <utility>

#include <iostream>

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Homogeneous.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/IO/Polyhedron_iostream.h>
#include <CGAL/Nef_polyhedron_3.h>
#include <CGAL/minkowski_sum_3.h>
#include <CGAL/IO/Nef_polyhedron_iostream_3.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/bounding_box.h>
#include <CGAL/convex_decomposition_3.h>
#include <CGAL/Bounded_kernel.h>
#include <CGAL/Nef_polyhedron_2.h>
#include <CGAL/convex_hull_3.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>

#include <CGAL/extract_mean_curvature_flow_skeleton.h>
#include <boost/foreach.hpp>

#include <trimesh/TriMesh.h>
#include <trimesh/TriMesh_algo.h>

#include <meshproc_csg/kdtree++/kdtree.hpp>

#include <shape_msgs/Mesh.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <meshproc_csg/typedefs.h>

#include <boost/smart_ptr.hpp>

#include <meshproc_csg/csg.h>

namespace cutplan
{

typedef meshproc_csg::Point tPointSpec;

typedef boost::shared_ptr<meshproc_csg::MeshEntry> MeshEntryPtr;
typedef boost::shared_ptr<meshproc_csg::MeshEntry const> MeshEntryConstPtr;

}

#endif
