#ifndef PART_HPP
#define PART_HPP

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
//#include <CGAL/Polyhedron_incremental_builder_3.h>
#include <CGAL/Aff_transformation_3.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
//#include <CGAL/intersections.h>
//#include <CGAL/Nef_polyhedron_3.h>
//#include <CGAL/convex_decomposition_3.h>
//#include <CGAL/IO/STL.h>
#include <fstream>


typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point;
typedef CGAL::Polyhedron_3<Kernel> Polyhedron;
typedef Kernel::Vector_3 Vector;
typedef Kernel::Triangle_3 Triangle;
typedef CGAL::Aff_transformation_3<Kernel> Transformation;
typedef CGAL::AABB_face_graph_triangle_primitive<Polyhedron> Primitive;
typedef CGAL::AABB_traits<Kernel, Primitive> AABB_traits;
typedef CGAL::AABB_tree<AABB_traits> AABB_tree;
//typedef CGAL::Nef_polyhedron_3<Kernel> Nef_polyhedron;

class Part
{

public:

    enum PART_TYPE {
        INTERNAL,
        EXTERNAL,
        SCREW
    };

    Part(std::shared_ptr<Polyhedron> mesh, PART_TYPE type, size_t id) : mesh_(mesh), type_(type), id_(id) {}

    void translate(Vector translation);

    bool collide(std::shared_ptr<Part> otherPart);

    std::shared_ptr<Polyhedron> createNegative(std::shared_ptr<Polyhedron> substrate);


    std::shared_ptr<Polyhedron>     getMesh()       { return mesh_; }
    PART_TYPE                       getPartType()   { return type_; }
    size_t                          getId()         { return id_; }


    Point                           getCentroid();

    //Copy constructor
    Part(const Part& other) : mesh_(std::make_shared<Polyhedron>(*other.mesh_)) {}

    // Deep Copy Function (returns a shared_ptr to the new Part)
    std::shared_ptr<Part> clone() const {
        return std::make_shared<Part>(*this);  // Uses copy constructor
    }

private:

    std::shared_ptr<Polyhedron> mesh_;

    PART_TYPE type_;

    size_t id_;

};

#endif