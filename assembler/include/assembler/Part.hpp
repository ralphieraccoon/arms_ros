#ifndef PART_HPP
#define PART_HPP

#include "assembler/MeshFunctions.hpp"

class Part
{
public:

    enum PART_TYPE {
        NONE,
        INTERNAL,
        EXTERNAL,
        SCREW
    };
    
    Part(std::shared_ptr<TopoDS_Shape> shape, PART_TYPE type, size_t id, std::string name) : shape_(shape), type_(type), id_(id), name_(name) {}

    Part() = default;

    //Copy constructor
    Part(const Part& other) : shape_(std::make_shared<TopoDS_Shape>(*other.shape_)), id_(other.id_), type_(other.type_), name_(other.name_), vacuum_grasp_position_(other.vacuum_grasp_position_) {}

    // Deep copy
    std::shared_ptr<Part> clone() const {
        return std::make_shared<Part>(*this); 
    }

    void translate(gp_Vec translation) {shape_ = std::make_shared<TopoDS_Shape>(TranslateShape(*shape_, translation));}

    bool collide(std::shared_ptr<Part> otherPart);

    void setCentroidPosition(gp_Pnt position) {shape_ = std::make_shared<TopoDS_Shape>(ShapeSetCentroid(*shape_, position));}

    void createNegativeAndPositionPart(std::vector<std::vector<bool>>& occupancy);

    void generateVacuumGraspPosition();

    void generatePPGGraspPosition();

    void saveShape(std::string filename) {SaveShapeAsSTL(*shape_, filename);}

    gp_Pnt                          getVacuumGrasp()    { return vacuum_grasp_position_; }
    gp_Pnt                          getCoM()            { return ShapeCenterOfMass(*shape_); }
    gp_Pnt                          getCentroid()       { return ShapeCentroid(*shape_); }
    Standard_Real                   getHighestPoint()   { return ShapeHighestPoint(*shape_); }

    PART_TYPE                       getType()           { return type_; }
    size_t                          getId()             { return id_; }
    std::string                     getName()           { return name_; }
    std::shared_ptr<TopoDS_Shape>   getShape()          { return shape_; }

private:

    TopoDS_Shape GenerateGripperPlate(gp_Dir normal, gp_Pnt center);

    std::shared_ptr<TopoDS_Shape> shape_;

    PART_TYPE type_;

    gp_Pnt vacuum_grasp_position_;  //Relative to CoM

    size_t id_;

    std::string name_;
};

#endif