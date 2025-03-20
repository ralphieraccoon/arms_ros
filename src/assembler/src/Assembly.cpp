#include "assembler/Assembly.hpp"

#include "assembler/Part.hpp"

#include <iostream>

Assembly::Assembly()
{
    std::cout << "Creating assembly" << std::endl;
}

std::vector<size_t> Assembly::getPartIds()
{
    std::vector<size_t> part_ids;

    for (std::shared_ptr<Part> part : parts_)
    {
        part_ids.push_back(part->getId());
    }

    return part_ids;
}

std::shared_ptr<Part> Assembly::getPartById(size_t id)
{
    for (std::shared_ptr<Part> part : parts_)
    {
        if (part->getId() == id)
            return part;
    }

    std::cerr << "No part of this ID present in assembly" << std::endl;

    return std::shared_ptr<Part>(new Part());
}

int Assembly::getNumInternalParts()
{
    int num_internal_parts = 0;

    for (std::shared_ptr<Part> part : parts_)
    {
        if (part->getType() == Part::INTERNAL)
            num_internal_parts ++;
    }

    return num_internal_parts;
}

void Assembly::placeOnPoint(Point point)
{
    //Find x,y center and lowest z point

    double xmin = std::numeric_limits<double>::max();
    double ymin = std::numeric_limits<double>::max();
    double zmin = std::numeric_limits<double>::max();
    double xmax = std::numeric_limits<double>::lowest();
    double ymax = std::numeric_limits<double>::lowest();
    double zmax = std::numeric_limits<double>::lowest();

    for (std::shared_ptr<Part> part : parts_)
    {
        BoundingBox bbox = meshBoundingBox(part->getMesh());

        xmin = std::min(xmin, bbox.xmin());
        ymin = std::min(ymin, bbox.ymin());
        zmin = std::min(zmin, bbox.zmin());
        xmax = std::max(xmax, bbox.xmax());
        ymax = std::max(ymax, bbox.ymax());
        zmax = std::max(zmax, bbox.zmax());
    }

    double lowest_z = zmin;

    Point center = Point((xmin + xmax) / 2.0, (ymin + ymax) / 2.0, (zmin + zmax) / 2.0);

    //Move al parts so that x,y lies on point.x and point.y, and lowest z point lies on zero (or adjust for bed heighgt)

    Vector translation = point - Point(center.x(), center.y(), lowest_z);

    for (std::shared_ptr<Part> part : parts_)
    {
        part->translate(translation);
    }
}
