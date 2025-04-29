#include "assembler/Assembly.hpp"

#include "assembler/Part.hpp"

#include <iostream>

#include <TopoDS_Compound.hxx>
#include <BRep_Builder.hxx>
#include <StlAPI_Writer.hxx>
#include <BRepMesh_IncrementalMesh.hxx>

Assembly::Assembly()
{
    std::cout << "Creating assembly" << std::endl;
}

void Assembly::saveAsSTL(std::string filename)
{
    std::cout << "Saving assembly as STL" << std::endl;

    TopoDS_Compound compound;
    BRep_Builder builder;
    builder.MakeCompound(compound);

    for (std::shared_ptr<Part> part : parts_)
    {
        std::cout << "Adding part to builder" << std::endl;

        builder.Add(compound, *(part->getShape()));
    }
        
    BRepMesh_IncrementalMesh mesher(compound, 0.1);

    StlAPI_Writer writer;
    writer.Write(compound, filename.c_str());

}


// void Assembly::saveAssemblyAsSTL(std::string filename)
// {
//     for (std::shared_ptr<Part> part : parts_) {

//         std::vector<SurfaceMesh::Vertex_index> vertex_indices;
        
//         // Copy vertices
//         for (auto v : part->getMesh()->ver  ->vertices()) {
//             Kernel::Point_3 p = mesh.point(v);
//             SurfaceMesh::Vertex_index new_v = merged_mesh.add_vertex(p);
//             vertex_map[v] = new_v;
//         }

//         // Copy faces
//         for (auto f : mesh.faces()) {
//             std::vector<SurfaceMesh::Vertex_index> new_vertices;
//             for (auto v : CGAL::vertices_around_face(mesh.halfedge(f), mesh)) {
//                 new_vertices.push_back(vertex_map[v]);
//             }
//             merged_mesh.add_face(new_vertices);
//         }
//     }


//     Polyhedron combined_mesh;
    
//     for (std::shared_ptr<Part> part : parts_) {
//         Polyhedron temp_mesh = *part->getMesh();
//         CGAL::Polygon_mesh_processing::append(combined_mesh, temp_mesh);
//     }

//     std::ofstream out(filename, std::ios::binary);
//     CGAL::write_STL(out, combined_mesh);
//     out.close();
// }

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

void Assembly::alignToPart(std::shared_ptr<Part> part)
{
    // std::shared_ptr<Part> thisPart = getPartById(part->getId());

    // if (thisPart == nullptr)
    // {
    //     std::cerr << "Corresponding part can't be found in assembly" << std::endl;
    //     return;
    // }

    // Vector delta = part->getCentroidPosition() - thisPart->getCentroidPosition();

    // for (std::shared_ptr<Part> part_ : parts_)
    // {
    //     part_->translate(delta);
    // }
}

//void Assembly::placeOnPoint(Point point)
//{
    // //Find x,y center and lowest z point
    // std::cout << std::endl << "Placing on point: " << point.x() << " " << point.y() << " " << point.z() << std::endl;


    // double xmin = std::numeric_limits<double>::max();
    // double ymin = std::numeric_limits<double>::max();
    // double zmin = std::numeric_limits<double>::max();
    // double xmax = std::numeric_limits<double>::lowest();
    // double ymax = std::numeric_limits<double>::lowest();
    // double zmax = std::numeric_limits<double>::lowest();

    // for (std::shared_ptr<Part> part : parts_)
    // {
    //     std::cout << "Part start pos: " << part->getCentroidPosition().x() << " " << part->getCentroidPosition().y() << " " << part->getCentroidPosition().z() << std::endl;


    //     BoundingBox bbox = meshBoundingBox(part->getMesh());

    //     xmin = std::min(xmin, bbox.xmin());
    //     ymin = std::min(ymin, bbox.ymin());
    //     zmin = std::min(zmin, bbox.zmin());
    //     xmax = std::max(xmax, bbox.xmax());
    //     ymax = std::max(ymax, bbox.ymax());
    //     zmax = std::max(zmax, bbox.zmax());
    // }

    // double lowest_z = zmin;

    // Point center = Point((xmin + xmax) / 2.0, (ymin + ymax) / 2.0, (zmin + zmax) / 2.0);

    // //Move al parts so that x,y lies on point.x and point.y, and lowest z point lies on zero (or adjust for bed heighgt)

    // Vector translation = point - Point(center.x(), center.y(), lowest_z);

    // for (std::shared_ptr<Part> part : parts_)
    // {
    //     part->translate(translation);

    //     std::cout << "Part end pos: " << part->getCentroidPosition().x() << " " << part->getCentroidPosition().y() << " " << part->getCentroidPosition().z() << std::endl;

    // }
//}
