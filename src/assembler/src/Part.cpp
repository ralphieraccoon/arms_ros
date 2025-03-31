#include "assembler/Part.hpp"

void Part::translate(Vector translation)
{   
    translateMesh(mesh_, translation);
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


Point Part::createNegative(std::shared_ptr<MeshObject> substrate, std::string filename)
{
    std::cout << "Creating negative" << std::endl;

    Point final_position(0, 0, 0);

    // Convert Polyhedra to Nef Polyhedra

    if (substrate == nullptr)
    {
        std::cout << "substrate null!" << std::endl;
        return final_position;
    }

    if (substrate->getMesh() == nullptr)
    {
        std::cout << "substrate mesh null!" << std::endl;
        return final_position;
    }

    //Copy the mesh and operate on the copy
    std::shared_ptr<Polyhedron> scaledMesh = std::make_shared<Polyhedron>(*mesh_);

    //Determine scaling factor from desired clearance
    double clearance = 1;

    BoundingBox bbox = meshBoundingBox(scaledMesh);

    auto x_size = bbox.x_span();

    auto y_size = bbox.y_span();

    auto z_size = bbox.z_span();

    auto target_x_size = x_size + clearance;

    auto target_y_size = y_size + clearance;
    
    auto target_z_size = z_size + clearance;

    auto x_scale_ratio = target_x_size / x_size;

    auto y_scale_ratio = target_y_size / y_size;

    auto z_scale_ratio = target_z_size / z_size;

    std::cout << "Scaling ratios: " <<  x_scale_ratio << " " << y_scale_ratio << " " << z_scale_ratio << std::endl;


    scaleMesh(scaledMesh, x_scale_ratio, y_scale_ratio, z_scale_ratio);   
                        //TODO we need to make the object orthogonally convex first! Maybe we just do this by hand for now
                        //TODO or maybe eve just convex? That might suffice for most objects


    //Find the lowest point of the substrate
    double substrate_lowest_z = meshLowestPoint(substrate->getMesh());

    //Find the lowest point of the part
    double part_lowest_z = meshLowestPoint(scaledMesh);

    //Move the part so that it's lowest point is 2mm above the lowest point of the substrate

    translateMesh(scaledMesh, Vector(0, 0, substrate_lowest_z + 2 - part_lowest_z));

    auto distance_moved = substrate_lowest_z + 2 - part_lowest_z;

    //Now, loop

    int i = 0;

    while (true)
    {
        std::cout << "Distance moved: " << distance_moved << std::endl;

        //Create the nefs and do the substraction

        SurfaceMesh mesh;
        CGAL::copy_face_graph(*(substrate->getMesh()), mesh);

        Nef_polyhedron nef_substrate(mesh);

        SurfaceMesh mesh2;
        CGAL::copy_face_graph(*scaledMesh, mesh2);

        Nef_polyhedron nef_part(mesh2);

        if (!nef_substrate.is_simple()) std::cout << "Error: Nef_substrate not simple!" << std::endl;
        if (!nef_part.is_simple()) std::cout << "Error: Nef_part not simple!" << std::endl;


        if (nef_substrate.is_empty()) std::cout << "Error: nef_substrate is empty!" << std::endl;
        if (nef_part.is_empty()) std::cout << "Error: nef_part is empty!" << std::endl;

        // Perform subtraction (poly1 - poly2)
        Nef_polyhedron result = nef_substrate.difference(nef_part);

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

        // std::stringstream ss;
        
        // ss << filename << "_partial_" << distance_moved << std::endl;

        // saveMesh(cradle, ss.str());

        //If there are no downward faces, save the mesh
        if (downwards_faces == 0)
        {
            //Save mesh
            saveMesh(cradle, filename);

            std::cout << std::endl << std::endl << "SAVED AT " << distance_moved << std::endl << std::endl;

            break;
        }

        translateMesh(scaledMesh, Vector(0, 0, 0.1));

        distance_moved += 0.1;
    }

    final_position += Vector(0, 0, distance_moved); 

    return final_position;
}