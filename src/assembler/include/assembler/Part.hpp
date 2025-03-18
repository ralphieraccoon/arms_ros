#ifndef PART_HPP
#define PART_HPP

#include <fstream>

#include "assembler/MeshFunctions.hpp"

class Substrate
{
public:

    Substrate() {}

    std::shared_ptr<Polyhedron>     getMesh()       { return mesh_; }

    void setMesh(std::shared_ptr<Polyhedron> mesh) { mesh_ = mesh; }

    void centerMesh();

private:

    std::shared_ptr<Polyhedron> mesh_;
};

class Part
{
public:

    enum PART_TYPE {
        INTERNAL,
        EXTERNAL,
        SCREW
    };

    Part(std::shared_ptr<Polyhedron> mesh, PART_TYPE type, size_t id, std::string name) : mesh_(mesh), type_(type), id_(id), name_(name) {}

    void translate(Vector translation);

    bool collide(std::shared_ptr<Part> otherPart);

    void createNegative(std::shared_ptr<Substrate> substrate, std::string filename);

    void centerMesh();

    std::shared_ptr<Polyhedron>     getMesh()       { return mesh_; }
    PART_TYPE                       getType()       { return type_; }
    size_t                          getId()         { return id_; }
    std::string                     getName()       { return name_; }


    Point                           getCentroid();

    //Copy constructor
    Part(const Part& other) : mesh_(std::make_shared<Polyhedron>(*other.mesh_)) {}

    // Deep Copy Function (returns a shared_ptr to the new Part)
    std::shared_ptr<Part> clone() const {
        return std::make_shared<Part>(*this);  // Uses copy constructor
    }

private:

    void debugMesh(std::shared_ptr<Polyhedron> mesh, std::string name = "polyhedron");

    void debugNefMesh(Nef_polyhedron mesh, std::string name = "nef_polyhedron");

    void saveMesh(std::shared_ptr<Polyhedron> mesh, std::string filename);

    void scaleMesh(std::shared_ptr<Polyhedron> mesh);

    std::shared_ptr<Polyhedron> subtractMesh(std::shared_ptr<Polyhedron> mesh1, std::shared_ptr<Polyhedron> mesh2);

    void removeOverhangs(std::shared_ptr<Polyhedron> mesh);

    std::shared_ptr<Polyhedron> mesh_;

    PART_TYPE type_;

    size_t id_;

    std::string name_;

};

class BuildTriangularPrism : public CGAL::Modifier_base<Polyhedron::HalfedgeDS> {
    std::vector<Point> vertices;
public:
    BuildTriangularPrism(const std::vector<Point>& v) : vertices(v) {}

    void operator()(Polyhedron::HalfedgeDS& hds) {
        CGAL::Polyhedron_incremental_builder_3<Polyhedron::HalfedgeDS> builder(hds, true);
        builder.begin_surface(6, 8); // 6 vertices, 8 faces

        // Add vertices
        for (const auto& v : vertices) {
            builder.add_vertex(v);
        }

        // Bottom face
        builder.begin_facet();
        builder.add_vertex_to_facet(1);
        builder.add_vertex_to_facet(2);
        builder.add_vertex_to_facet(0);
        builder.end_facet();

        //Top face
        builder.begin_facet();
        builder.add_vertex_to_facet(5);
        builder.add_vertex_to_facet(4);
        builder.add_vertex_to_facet(3);
        builder.end_facet();

        //Side faces with bottom edge
        builder.begin_facet();
        builder.add_vertex_to_facet(2);
        builder.add_vertex_to_facet(1);
        builder.add_vertex_to_facet(4);
        builder.end_facet();

        builder.begin_facet();
        builder.add_vertex_to_facet(1);
        builder.add_vertex_to_facet(0);
        builder.add_vertex_to_facet(3);
        builder.end_facet();

        builder.begin_facet();
        builder.add_vertex_to_facet(0);
        builder.add_vertex_to_facet(2);
        builder.add_vertex_to_facet(5);
        builder.end_facet();

        //Side faces with top edge
        builder.begin_facet();
        builder.add_vertex_to_facet(3);
        builder.add_vertex_to_facet(4);
        builder.add_vertex_to_facet(1);
        builder.end_facet();

        builder.begin_facet();
        builder.add_vertex_to_facet(4);
        builder.add_vertex_to_facet(5);
        builder.add_vertex_to_facet(2);
        builder.end_facet();

        builder.begin_facet();
        builder.add_vertex_to_facet(5);
        builder.add_vertex_to_facet(3);
        builder.add_vertex_to_facet(0);
        builder.end_facet();

        builder.end_surface();
    }
};

#endif