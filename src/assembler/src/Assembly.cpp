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
    //Move al parts so that x,y lies on point.x and point.y, and lowest z point lies on zero (or adjust for bed heighgt)

    //TODO
}
