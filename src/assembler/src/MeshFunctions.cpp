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

double meshLowestPoint(std::shared_ptr<Polyhedron> mesh)
{
    if (mesh->size_of_facets() == 0)
    {
        std::cout << "Mesh has no facets" << std::endl;
        return 0;
    }

    return meshBoundingBox(mesh).zmin();
}

bool saveMesh(std::shared_ptr<Polyhedron> mesh, std::string filename)
{
    std::ofstream out(filename);
    if (!out) {
        throw std::runtime_error("Failed to open output file.");
        return false;
    }

    out << "solid polyhedron\n";
    for (auto f = mesh->facets_begin(); f != mesh->facets_end(); ++f) {
        // Get three vertices of the face (assuming triangular faces)
        auto h = f->facet_begin();
        Point p1 = h->vertex()->point();
        Point p2 = (++h)->vertex()->point();
        Point p3 = (++h)->vertex()->point();

        // Compute a simple normal (not necessarily unit length)
        Kernel::Vector_3 normal = CGAL::cross_product(p2 - p1, p3 - p1);

        out << "  facet normal " << normal.x() << " " << normal.y() << " " << normal.z() << "\n";
        out << "    outer loop\n";
        out << "      vertex " << p1.x() << " " << p1.y() << " " << p1.z() << "\n";
        out << "      vertex " << p2.x() << " " << p2.y() << " " << p2.z() << "\n";
        out << "      vertex " << p3.x() << " " << p3.y() << " " << p3.z() << "\n";
        out << "    endloop\n";
        out << "  endfacet\n";
    }
    out << "endsolid polyhedron\n";

    out.close();

    return true;
}

void debugMesh(std::shared_ptr<Polyhedron> poly, std::string name)
{
    std::cout << std::endl << name << " Information:\n";
    
    // Print total number of vertices and faces
    std::cout << "Total vertices: " << poly->size_of_vertices() << "\n";
    std::cout << "Total faces: " << poly->size_of_facets() << "\n";

    // Create a mapping from vertex handles to indices
    std::map<Polyhedron::Vertex_const_handle, int> vertex_index;
    int index = 0;
    
    // Print indexed list of vertices
    std::cout << "\nVertices (index: x, y, z):\n";
    for (auto v = poly->vertices_begin(); v != poly->vertices_end(); ++v, ++index) {
        vertex_index[v] = index;
        std::cout << index << ": (" << v->point().x() << ", " 
                                  << v->point().y() << ", " 
                                  << v->point().z() << ")\n";
    }

    // Print faces with indexed vertices
    std::cout << "\nFaces (vertex indices):\n";
    index = 0;
    for (auto f = poly->facets_begin(); f != poly->facets_end(); ++f, ++index) {
        std::cout << "Face " << index << ": ";

        auto normal = CGAL::Polygon_mesh_processing::compute_face_normal(f, *poly);
            std::cout << "Facet normal: " 
                      << normal.x() << " " 
                      << normal.y() << " " 
                      << normal.z() << std::endl;
        // Iterate over the halfedges of the face
        auto h = f->halfedge();
        do {
            std::cout << vertex_index[h->vertex()] << " ";
            h = h->next();
        } while (h != f->halfedge());
        
        std::cout << "\n";
    }
}

void debugNefMesh(Nef_polyhedron mesh, std::string name)
{
    std::cout << std::endl << "Nef mesh: " << name << std::endl;

    for (Nef_polyhedron::SFace_const_iterator sface_it = mesh.sfaces_begin(); sface_it != mesh.sfaces_end(); ++sface_it) {
        std::cout << "New facet: " << std::endl;

        // Iterate over the halfedges that form the boundary of the facet
        Nef_polyhedron::SHalfedge_const_handle she = sface_it->sface_cycles_begin();
        if (she == nullptr) continue; // Skip empty faces

        Nef_polyhedron::SHalfedge_const_handle start = she; // Store start to detect loops
        do {
            Kernel::Point_3 p = she->source()->point();
            std::cout << "  Vertex: (" << CGAL::to_double(p.x()) << ", "
                                        << CGAL::to_double(p.y()) << ", "
                                        << CGAL::to_double(p.z()) << ")" << std::endl;

            she = she->next(); // Move to next halfedge in the facet loop
        } while (she != start);
    }

    // for (auto f = result.facets_begin(); f != cradle->facets_end(); ++f) {
    //     // Get three vertices of the face (assuming triangular faces)
    //     auto h = f->facet_begin();
    //     Point p1 = h->vertex()->point();
    //     Point p2 = (++h)->vertex()->point();
    //     Point p3 = (++h)->vertex()->point();

    //     // Compute a simple normal (not necessarily unit length)
    //     Kernel::Vector_3 normal = CGAL::cross_product(p2 - p1, p3 - p1);

    //     out << "  facet normal " << normal.x() << " " << normal.y() << " " << normal.z() << "\n";
    //     out << "    outer loop\n";
    //     out << "      vertex " << p1.x() << " " << p1.y() << " " << p1.z() << "\n";
    //     out << "      vertex " << p2.x() << " " << p2.y() << " " << p2.z() << "\n";
    //     out << "      vertex " << p3.x() << " " << p3.y() << " " << p3.z() << "\n";
    //     out << "    endloop\n";
    //     out << "  endfacet\n";
    // }



    // for (auto it = result.vertices_begin(); it != result.vertices_end(); ++it) {
    //     const Point& p = it->point();
    //     std::cout << "Vertex: (" << CGAL::to_double(p.x()) << ", " 
    //               << CGAL::to_double(p.y()) << ", " 
    //               << CGAL::to_double(p.z()) << ")\n";
    // }
}

void positionMesh(std::shared_ptr<Polyhedron> mesh, Point target_position)
{
    Point initial_center = meshCenter(mesh);

    Vector delta = target_position - initial_center;

    //Substract initial center from each vertex to create new center of 0,0,0

    for (auto vertex = mesh->vertices_begin(); vertex != mesh->vertices_end(); vertex++)
    {
        vertex->point() += delta;
    }
}

Point meshCenter(std::shared_ptr<Polyhedron> mesh)
{
    if (mesh == nullptr)
        return Point(0, 0, 0);

    if (mesh->size_of_vertices() == 0)
        return Point(0, 0, 0);

    if (!mesh->is_closed())
    {
        std::cout << "Can't find centroid of open mesh" << std::endl;

        return Point(0, 0, 0);
    }

    BoundingBox bbox = meshBoundingBox(mesh);

    double cx = (bbox.xmin() + bbox.xmax()) / 2.0;
    double cy = (bbox.ymin() + bbox.ymax()) / 2.0;
    double cz = (bbox.zmin() + bbox.zmax()) / 2.0;

    return Point(cx, cy, cz);

    //Iterate through each vertex, find the maximum and minimum along each axis, shift vertex positions so that the centorid lies at 0,0,0

    // Point max = mesh->vertices_begin()->point();

    // Point min = mesh->vertices_begin()->point();

    // for (auto vertex = mesh->vertices_begin(); vertex != mesh->vertices_end(); vertex++)
    // {
    //     Point point = vertex->point();

    //     if (point.x() > max.x())
    //         max = Point(point.x(), max.y(), max.z());

    //     if (point.y() > max.y())
    //         max = Point(max.x(), point.y(), max.z());

    //     if (point.z() > max.z())
    //         max = Point(max.x(), max.y(), point.z());

    //     if (point.x() < min.x())
    //         min = Point(point.x(), min.y(), min.z());

    //     if (point.y() < min.y())
    //         min = Point(min.x(), point.y(), min.z());

    //     if (point.z() < min.z())
    //         min = Point(min.x(), min.y(), point.z());
    // }

    // Vector size = max - min;

    // Point center = min + 0.5 * size;

    // return center;
}

void scaleMesh(std::shared_ptr<Polyhedron> mesh)
{
    Transformation scaleTransform(CGAL::SCALING, 1.05);

    for (auto v = mesh->points_begin(); v != mesh->points_end(); ++v) {
        *v = scaleTransform(*v);  // Apply transformation
    }
}

BoundingBox meshBoundingBox(std::shared_ptr<Polyhedron> mesh)
{
    if (mesh->size_of_vertices() == 0)
        return CGAL::Bbox_3(0, 0, 0, 0, 0, 0);  // Return empty bbox if mesh is empty

    double xmin = std::numeric_limits<double>::max();
    double ymin = std::numeric_limits<double>::max();
    double zmin = std::numeric_limits<double>::max();
    double xmax = std::numeric_limits<double>::lowest();
    double ymax = std::numeric_limits<double>::lowest();
    double zmax = std::numeric_limits<double>::lowest();

    for (auto v = mesh->vertices_begin(); v != mesh->vertices_end(); ++v) {
        const Point& p = v->point();
        xmin = std::min(xmin, CGAL::to_double(p.x()));
        ymin = std::min(ymin, CGAL::to_double(p.y()));
        zmin = std::min(zmin, CGAL::to_double(p.z()));
        xmax = std::max(xmax, CGAL::to_double(p.x()));
        ymax = std::max(ymax, CGAL::to_double(p.y()));
        zmax = std::max(zmax, CGAL::to_double(p.z()));
    }

    return CGAL::Bbox_3(xmin, ymin, zmin, xmax, ymax, zmax);
}