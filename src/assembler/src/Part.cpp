#include "assembler/Part.hpp"

void Part::translate(Vector translation)
{   
    Transformation moveTransform(CGAL::TRANSLATION, translation);

    for (auto v = mesh_->points_begin(); v != mesh_->points_end(); ++v) {
        *v = moveTransform(*v);  // Apply transformation
    }
}

bool Part::collide(std::shared_ptr<Part> otherPart)
{
    //AABB_tree tree1(faces(*mesh_).first, faces(*mesh_).second, *mesh_);
    AABB_tree tree2(faces(*(otherPart->getMesh())).first, faces(*(otherPart->getMesh())).second, *(otherPart->getMesh()));

    //tree1.accelerate_distance_queries();
    tree2.accelerate_distance_queries();

    // Iterate through the faces of one polyhedron
    for (auto f1 = mesh_->facets_begin(); f1 != mesh_->facets_end(); ++f1) {
        // Convert face into a triangle
        auto h = f1->halfedge();
        Triangle tri1(h->vertex()->point(), h->next()->vertex()->point(), h->next()->next()->vertex()->point());

        // Check if this triangle intersects with tree2
        if (tree2.do_intersect(tri1)) {
            return true;  // Collision detected
        }
    }

    return false;  // No collision
}

void Part::centerMesh()
{
    if (mesh_ == nullptr)
        return;

    if (mesh_->size_of_vertices() == 0)
        return;


    //Iterate through each vertex, find the maximum and minimum along each axis, shift vertex positions so that the centorid lies at 0,0,0

    Point max = mesh_->vertices_begin()->point();

    Point min = mesh_->vertices_begin()->point();

    for (auto vertex = mesh_->vertices_begin(); vertex != mesh_->vertices_end(); vertex++)
    {
        Point point = vertex->point();

        if (point.x() > max.x())
            max = Point(point.x(), max.y(), max.z());

        if (point.y() > max.y())
            max = Point(max.x(), point.y(), max.z());

        if (point.z() > max.z())
            max = Point(max.x(), max.y(), point.z());

        if (point.x() < min.x())
            min = Point(point.x(), min.y(), min.z());

        if (point.y() < min.y())
            min = Point(min.x(), point.y(), min.z());

        if (point.z() < min.z())
            min = Point(min.x(), min.y(), point.z());
    }

    Vector size = max - min;

    Point initial_center = min + 0.5 * size;

    Vector delta = Point(0, 0, 0) - initial_center;

    //Substract initial center from each vertex to create new center of 0,0,0

    for (auto vertex = mesh_->vertices_begin(); vertex != mesh_->vertices_end(); vertex++)
    {
        vertex->point() += delta;
    }
}

void Part::createNegative(std::shared_ptr<Substrate> substrate, std::string filename)
{
    std::cout << "Creating negative" << std::endl;

    // Convert Polyhedra to Nef Polyhedra

    if (substrate == nullptr)
    {
        std::cout << "substrate null!" << std::endl;
        return;
    }

    if (substrate->getMesh() == nullptr)
    {
        std::cout << "substrate mesh null!" << std::endl;
        return;
    }

    SurfaceMesh mesh;
    CGAL::copy_face_graph(*(substrate->getMesh()), mesh);

    Nef_polyhedron nef_substrate(mesh);


    //Nef_polyhedron nef_substrate(*substrate->getMesh());

    std::cout << "Nef 1 created" << std::endl;


    SurfaceMesh mesh2;
    CGAL::copy_face_graph(*mesh_, mesh2);

    Nef_polyhedron nef_part(mesh2);

    //Nef_polyhedron nef_part(*mesh_);

    std::cout << "Nef 2 created" << std::endl;


    if (!nef_substrate.is_simple()) std::cout << "Error: Nef_substrate not simple!" << std::endl;
    if (!nef_part.is_simple()) std::cout << "Error: Nef_part not simple!" << std::endl;


    if (nef_substrate.is_empty()) std::cout << "Error: nef_substrate is empty!" << std::endl;
    if (nef_part.is_empty()) std::cout << "Error: nef_part is empty!" << std::endl;

    // Perform subtraction (poly1 - poly2)
    Nef_polyhedron result = nef_substrate.difference(nef_part);

    std::cout << "Nefs subtracted" << std::endl;

    result.regularization();  // Improves numerical stability
    result.simplify();        // Removes unnecessary small faces

    // std::ofstream out1("result.off");
    // out1 << result;
    // out1.close();

    // Convert back to Polyhedron
    std::shared_ptr<Polyhedron> cradle = std::shared_ptr<Polyhedron>(new Polyhedron());
    if (result.is_simple()) {
        result.convert_to_polyhedron(*cradle);
    } else {
        throw std::runtime_error("Resulting shape is not a valid polyhedron.");
    }

    if (cradle->is_closed())
        std::cout << "Cradle mesh is closed!" << std::endl;

    else
        std::cout << "Cradle mesh is open!" << std::endl;

    std::ofstream out(filename);
    if (!out) {
        throw std::runtime_error("Failed to open output file.");
    }

    out << "solid polyhedron\n";
    for (auto f = cradle->facets_begin(); f != cradle->facets_end(); ++f) {
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
}

Point Part::getCentroid()
{
    //Point centroid(0, 0, 0);

    //if (!mesh_->is_closed())
    //{
    //    std::cout << "Can't find centroid of open mesh" << std::endl;

    //    return centroid;
    //}



    Vector weightedSum(0, 0, 0);
    double totalVolume = 0.0;
    Point origin(0, 0, 0);  // Use origin as reference for tetrahedra

    // // Iterate through all faces
    // for (auto f = mesh_->facets_begin(); f != mesh_->facets_end(); ++f) 
    // {
    //     // Get the three vertices of the face
    //     auto h = f->halfedge();
    //     Point p1 = h->vertex()->point();
    //     Point p2 = h->next()->vertex()->point();
    //     Point p3 = h->next()->next()->vertex()->point();

    //     // Compute tetrahedron volume using determinant formula
    //     double volume = CGAL::determinant(p1 - CGAL::ORIGIN, p2 - CGAL::ORIGIN, p3 - CGAL::ORIGIN) / 6.0;
    //     if (volume == 0) continue;  // Skip degenerate tetrahedra

    //     totalVolume += volume;

    //     // Compute tetrahedron centroid
    //     Point centroid = CGAL::centroid(origin, p1, p2, p3);

    //     // Accumulate volume-weighted centroid contributions
    //     weightedSum = weightedSum + (volume * (centroid - CGAL::ORIGIN));

    // }

    // return CGAL::ORIGIN + (weightedSum / totalVolume);


    return origin;

    // Final center of mass computation

    //return CGAL::ORIGIN + (Point(weightedSum.x() / totalVolume, weightedSum.y() / totalVolume, weightedSum.z() / totalVolume));
}

void Substrate::centerMesh()
{
    if (mesh_ == nullptr)
        return;

    if (mesh_->size_of_vertices() == 0)
        return;


    //Iterate through each vertex, find the maximum and minimum along each axis, shift vertex positions so that the centorid lies at 0,0,0

    Point max = mesh_->vertices_begin()->point();

    Point min = mesh_->vertices_begin()->point();

    for (auto vertex = mesh_->vertices_begin(); vertex != mesh_->vertices_end(); vertex++)
    {
        Point point = vertex->point();

        if (point.x() > max.x())
            max = Point(point.x(), max.y(), max.z());

        if (point.y() > max.y())
            max = Point(max.x(), point.y(), max.z());

        if (point.z() > max.z())
            max = Point(max.x(), max.y(), point.z());

        if (point.x() < min.x())
            min = Point(point.x(), min.y(), min.z());

        if (point.y() < min.y())
            min = Point(min.x(), point.y(), min.z());

        if (point.z() < min.z())
            min = Point(min.x(), min.y(), point.z());
    }

    Vector size = max - min;

    Point initial_center = min + 0.5 * size;

    Vector delta = Point(0, 0, 0) - initial_center;

    //Substract initial center from each vertex to create new center of 0,0,0

    for (auto vertex = mesh_->vertices_begin(); vertex != mesh_->vertices_end(); vertex++)
    {
        vertex->point() += delta;
    }
}