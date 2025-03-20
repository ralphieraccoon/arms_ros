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

    scaleMesh(mesh_);   //TODO Need to copy before scaling in future
                        //TODO we need to make the object orthogonally convex first! Maybe we just do this by hand for now
                        //TODO or maybe eve just convex? That might suffice for most objects


    //Find the lowest point of the substrate
    double substrate_lowest_z = meshLowestPoint(substrate->getMesh());

    //Find the lowest point of the part
    double part_lowest_z = meshLowestPoint(mesh_);

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

        translate(Vector(0, 0, 0.1));

        distance_moved += 0.1;
    }

    //Reset the part back to its original position
    translate(Vector(0, 0, -distance_moved));

    final_position += Vector(0, 0, distance_moved); 

    return final_position;
}