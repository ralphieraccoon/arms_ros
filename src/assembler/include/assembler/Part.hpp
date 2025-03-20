#ifndef PART_HPP
#define PART_HPP

#include <fstream>

#include "assembler/MeshFunctions.hpp"


class MeshObject
{

public:

    MeshObject() {}

    MeshObject(std::shared_ptr<Polyhedron> mesh) : mesh_(mesh) {}

    MeshObject(const MeshObject& other) : mesh_(std::make_shared<Polyhedron>(*other.mesh_)) {}

    std::shared_ptr<Polyhedron>     getMesh()       { return mesh_; }

    Point                           getCentroidPosition() { return meshCenter(mesh_); }

    Point getMinBounds();

    Point getMaxBounds();

    void setMesh(std::shared_ptr<Polyhedron> mesh) { mesh_ = mesh; }

    void setCentroidPosition(Point position) { positionMesh(mesh_, position); }

protected:

    std::shared_ptr<Polyhedron> mesh_;
};   


class Part : public MeshObject
{
public:

    enum PART_TYPE {
        INTERNAL,
        EXTERNAL,
        SCREW
    };

    Part(std::shared_ptr<Polyhedron> mesh, PART_TYPE type, size_t id, std::string name) : MeshObject(mesh), type_(type), id_(id), name_(name) {}

    Part() {}

    void translate(Vector translation);

    bool collide(std::shared_ptr<Part> otherPart);

    Point createNegative(std::shared_ptr<MeshObject> substrate, std::string filename);

    PART_TYPE                       getType()       { return type_; }
    size_t                          getId()         { return id_; }
    std::string                     getName()       { return name_; }

    //Copy constructor
    Part(const Part& other) : MeshObject(other) {}

    // Deep Copy Function (returns a shared_ptr to the new Part)
    std::shared_ptr<Part> clone() const {
        return std::make_shared<Part>(*this);  // Uses copy constructor
    }

private:

    PART_TYPE type_;

    size_t id_;

    std::string name_;

};

#endif