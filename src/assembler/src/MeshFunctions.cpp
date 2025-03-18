#include "assembler/MeshFunctions.hpp"

Point facetLowestPoint(Polyhedron::Facet_handle facet)
{
    auto halfedge = facet->halfedge();  // Get one halfedge of the facet
    auto start = halfedge;  // Store the starting halfedge

    auto lowest_z = halfedge->vertex()->point().z();

    Point lowest_point = halfedge->vertex()->point();

    do {
        auto z = halfedge->vertex()->point().z();
        
        if (z < lowest_z)
        {
            lowest_z = z;
            lowest_point = halfedge->vertex()->point();
        }

        halfedge = halfedge->next();  // Move to the next halfedge in the facet
    } while (halfedge != start);  // Stop when we loop back to the start

    return lowest_point;
}

Point meshLowestPoint(std::shared_ptr<Polyhedron> mesh)
{
    if (mesh->size_of_facets() == 0)
    {
        std::cout << "Mesh has no facets" << std::endl;
        return Point(0, 0, 0);
    }

    Point lowest_point = mesh->facets_begin()->halfedge()->vertex()->point();

    for (auto facet = mesh->facets_begin(); facet != mesh->facets_end(); facet++)
    {
        auto point = facetLowestPoint(facet);

        if (point.z() < lowest_point.z())
            lowest_point = point;
    }

    return lowest_point;
}