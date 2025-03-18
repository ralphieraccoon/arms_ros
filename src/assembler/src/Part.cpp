#include "assembler/Part.hpp"

void Part::translate(Vector translation)
{   
    Transformation moveTransform(CGAL::TRANSLATION, translation);

    for (auto v = mesh_->points_begin(); v != mesh_->points_end(); ++v) {
        *v = moveTransform(*v);  // Apply transformation
    }
}

void Part::scaleMesh(std::shared_ptr<Polyhedron> mesh)
{
    Transformation scaleTransform(CGAL::SCALING, 1.05);

    for (auto v = mesh->points_begin(); v != mesh->points_end(); ++v) {
        *v = scaleTransform(*v);  // Apply transformation
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

    scaleMesh(mesh_);   //TODO Need to copy before scaling in future
                        //TODO we need to make the object orthogonally convex first! Maybe we just do this by hand for now
                        //TODO or maybe eve just convex? That might suffice for most objects


    //Find the lowest point of the substrate
    auto substrate_lowest_z = meshLowestPoint(substrate->getMesh()).z();

    //Find the lowest point of the part
    auto part_lowest_z = meshLowestPoint(mesh_).z();

    //Move the part so that it's lowest point is 2mm above the lowest point of the substrate

    translate(Vector(0, 0, substrate_lowest_z + 2 - part_lowest_z));

    auto distance_moved = substrate_lowest_z + 2 - part_lowest_z;

    //Now, loop

    int i = 0;

    while (true)
    {
        //Create the nefs and do the substraction

        SurfaceMesh mesh;
        CGAL::copy_face_graph(*(substrate->getMesh()), mesh);

        Nef_polyhedron nef_substrate(mesh);

        //Nef_polyhedron nef_substrate(*substrate->getMesh());

        std::cout << "Nef 1 created" << std::endl;

        //debugNefMesh(nef_substrate, "Substrate");

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

        // Convert back to Polyhedron
        std::shared_ptr<Polyhedron> cradle = std::shared_ptr<Polyhedron>(new Polyhedron());
        if (result.is_simple()) {
            result.convert_to_polyhedron(*cradle);
        } else {
            throw std::runtime_error("Resulting shape is not a valid polyhedron.");
        }


        //Check if there are any faces pointing downwards
        int downwards_faces = 0;


        for (auto facet = cradle->facets_begin(); facet != cradle->facets_end(); facet++)
        {
            Vector normal = CGAL::Polygon_mesh_processing::compute_face_normal(facet, *cradle);

            if (normal.z() < -0.001 && facetLowestPoint(facet).z() > substrate_lowest_z + 1) {  // Face is downward-facing
                downwards_faces++;
            }
        }


        std::cout << "Num downwards faces: " << downwards_faces << std::endl;

        //If there are no downward faces, save the mesh
        if (downwards_faces == 0)
        {
            //Save mesh
            saveMesh(cradle, filename);

            std::cout << std::endl << std::endl << "SAVED AT " << distance_moved << std::endl << std::endl;

            break;
        }

        //std::stringstream ss;

        //ss << "partial_at_" << distance_moved << "_" << filename;

        //saveMesh(cradle, ss.str());

        //If there are, move the part up 1mm

        translate(Vector(0, 0, 0.1));

        distance_moved += 0.1;
    }

    //Reset the part back to its original position
    translate(Vector(0, 0, -distance_moved));



    return;

    SurfaceMesh mesh;
    CGAL::copy_face_graph(*(substrate->getMesh()), mesh);

    Nef_polyhedron nef_substrate(mesh);


    //Nef_polyhedron nef_substrate(*substrate->getMesh());

    std::cout << "Nef 1 created" << std::endl;

    debugNefMesh(nef_substrate, "Substrate");


    SurfaceMesh mesh2;
    CGAL::copy_face_graph(*mesh_, mesh2);

    Nef_polyhedron nef_part(mesh2);

    //Nef_polyhedron nef_part(*mesh_);

    std::cout << "Nef 2 created" << std::endl;

    debugNefMesh(nef_part, "Part");


    if (!nef_substrate.is_simple()) std::cout << "Error: Nef_substrate not simple!" << std::endl;
    if (!nef_part.is_simple()) std::cout << "Error: Nef_part not simple!" << std::endl;


    if (nef_substrate.is_empty()) std::cout << "Error: nef_substrate is empty!" << std::endl;
    if (nef_part.is_empty()) std::cout << "Error: nef_part is empty!" << std::endl;

    // Perform subtraction (poly1 - poly2)
    Nef_polyhedron result = nef_substrate.difference(nef_part);

    std::cout << "Nefs subtracted" << std::endl;

    result.regularization();  // Improves numerical stability
    result.simplify();        // Removes unnecessary small faces

    debugNefMesh(result, "Cradle");

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


    debugMesh(cradle, "Cradle");


    //Now remove overhangs

    // std::vector<Polyhedron::Facet_handle> faces_to_remove;

    // for (auto facet = cradle->facets_begin(); facet != cradle->facets_end(); facet++)
    // {
    //     Vector normal = CGAL::Polygon_mesh_processing::compute_face_normal(facet, *cradle);
    //     if (normal.z() < 0) {  // Face is downward-facing
    //         faces_to_remove.push_back(facet);
    //     }
    // }





    // // // Iterate over all faces
    // // for (Polyhedron::Facet_handle f : cradle->facets()) {
    // //     Vector normal = CGAL::Polygon_mesh_processing::compute_face_normal(f, *cradle);
    // //     if (normal.z() < 0) {  // Face is downward-facing
    // //         faces_to_remove.push_back(f);
    // //     }
    // // }

    // // Remove faces iteratively
    // for (Polyhedron::Facet_handle f : faces_to_remove) {
    //     //CGAL::Euler::remove_face()
    //     CGAL::Euler::remove_face<Polyhedron>(f->halfedge(), *cradle);
    // }
    // cradle->normalize_border();  // Ensure the mesh remains valid

    removeOverhangs(cradle);

    saveMesh(cradle, filename);
}

void Part::saveMesh(std::shared_ptr<Polyhedron> mesh, std::string filename)
{
    std::ofstream out(filename);
    if (!out) {
        throw std::runtime_error("Failed to open output file.");
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
}

void Part::debugMesh(std::shared_ptr<Polyhedron> poly, std::string name)
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

void Part::debugNefMesh(Nef_polyhedron mesh, std::string name)
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

std::shared_ptr<Polyhedron> Part::subtractMesh(std::shared_ptr<Polyhedron> poly1, std::shared_ptr<Polyhedron> poly2)
{
    std::cout << std::endl << "Substracting meshes" << std::endl;

    SurfaceMesh mesh;

    CGAL::copy_face_graph(*poly1, mesh);

    Nef_polyhedron nef_substrate(mesh);


    SurfaceMesh mesh2;

    CGAL::copy_face_graph(*poly2, mesh2);

    Nef_polyhedron nef_object(mesh2);

    debugNefMesh(nef_substrate, "Substrate");

    debugNefMesh(nef_object, "Object");


    if (!nef_substrate.is_simple()) std::cout << "Error: Nef_substrate not simple!" << std::endl;
    if (!nef_object.is_simple()) std::cout << "Error: Nef_prism not simple!" << std::endl;


    if (nef_substrate.is_empty()) std::cout << "Error: nef_substrate is empty!" << std::endl;
    if (nef_object.is_empty()) std::cout << "Error: nef_prism is empty!" << std::endl;

    // Perform subtraction (poly1 - poly2)
    Nef_polyhedron result = nef_substrate.difference(nef_object);

    std::cout << "Nefs subtracted" << std::endl;

    result.regularization();  // Improves numerical stability
    result.simplify();        // Removes unnecessary small faces

    std::cout << "Result: " << std::endl;

    debugNefMesh(result, "Result");

    std::shared_ptr<Polyhedron> polyhedron = std::shared_ptr<Polyhedron>(new Polyhedron());

    // Convert back to Polyhedron
    if (result.is_simple()) {
        result.convert_to_polyhedron(*polyhedron);
    } else {
        throw std::runtime_error("Resulting shape is not a valid polyhedron.");
    }

    return polyhedron;
}

void Part::removeOverhangs(std::shared_ptr<Polyhedron> polyhedron)
{
    //Find lowest point
    auto vertex_it = polyhedron->vertices_begin();
    
    // Start with the first vertex
    Point lowest_point = vertex_it->point();
    auto min_z = lowest_point.z();
    
    // Iterate through all vertices
    for (; vertex_it != polyhedron->vertices_end(); ++vertex_it) {
        const Point& p = vertex_it->point();
        if (p.z() < min_z) {
            min_z = p.z();
            lowest_point = p;
        }
    }

    //std::cout << "Lowest overall z: " << min_z << std::endl;

    //Find the faces for which the normal points downwards - but skip those faces that are very close to the bottom
    std::vector<Polyhedron::Facet_handle> faces_to_remove;

    for (auto facet = polyhedron->facets_begin(); facet != polyhedron->facets_end(); facet++)
    {
        Vector normal = CGAL::Polygon_mesh_processing::compute_face_normal(facet, *polyhedron);

        //std::cout << "Lowest: " << facetLowestPoint(facet).z() << std::endl;


        if (normal.z() < 0 && facetLowestPoint(facet).z() > min_z + 1) {  // Face is downward-facing
            faces_to_remove.push_back(facet);
            //std::cout << "adding" << std::endl;
        }
    }

    std::cout << "Num faces to remove: " << faces_to_remove.size() << std::endl;

    //Iterate through each face

    int i = 0;

    for (Polyhedron::Facet_handle facet : faces_to_remove)
    {
        //Find the z position of the lowest vertex
        auto lowest_z = facetLowestPoint(facet).z();
        


        std::cout << "Lowest z: " << lowest_z << std::endl;
        
        std::cout << "Min z + 1: " << min_z + 1 << std::endl;

        if (lowest_z < min_z + 1)
            continue;

        std::cout << "Not continuing" << std::endl;


        std::vector<Point> prism_points;

        //Project the face onto the x-y plane at the height of the lowest z vertex, and find the points for the prism

        auto halfedge = facet->halfedge();  // Get one halfedge of the facet
        auto start = halfedge;  // Store the starting halfedge

        do {

            Point new_point(halfedge->vertex()->point().x(), halfedge->vertex()->point().y(), lowest_z);

            prism_points.push_back(new_point);

            halfedge = halfedge->next();  // Move to the next halfedge in the facet
        } while (halfedge != start);  // Stop when we loop back to the start


        

        if (prism_points.size() != 3)
        {
            std::cout << "Prism construciton error" << std::endl;
            return;
        }

        auto highest_z = lowest_z + 6;    //TODO

        prism_points.push_back(Point(prism_points[0].x(), prism_points[0].y(), highest_z));
        prism_points.push_back(Point(prism_points[1].x(), prism_points[1].y(), highest_z));
        prism_points.push_back(Point(prism_points[2].x(), prism_points[2].y(), highest_z));

        //Create a triangular prism from the project triangle, that extends above the top of the substrate/part mesh

        std::shared_ptr<Polyhedron> prism = std::shared_ptr<Polyhedron>(new Polyhedron());

        BuildTriangularPrism prismBuilder(prism_points);
        prism->delegate(prismBuilder);

        //Scale the prism up slightly - TODO

        scaleMesh(prism);


        debugMesh(prism, "Prism");

        debugMesh(polyhedron, "before_prism");


        //saveMesh(prism, "prism.stl");

        //Subtract the prism from the mesh

        polyhedron = subtractMesh(polyhedron, prism);
        
        std::stringstream ss;

        ss << "prism_removed_" << i << ".stl" << std::endl; 

        debugMesh(polyhedron, ss.str());

        saveMesh(polyhedron, ss.str());

        if (polyhedron->is_closed())
            std::cout << "Polyhedron mesh is closed!" << std::endl;

        else
            std::cout << "Polyhedron mesh is open!" << std::endl;

        i ++;
    }

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