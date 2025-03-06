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

