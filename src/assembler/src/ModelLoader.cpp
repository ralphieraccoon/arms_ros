#include "assembler/ModelLoader.hpp"
#include "assembler/Assembly.hpp"

#include <iostream>

//TODO Just load the single aiScene as before and have an assembly class that points to a list of meshes in that assembly and just make sure to put them
//back in the correct position after wiggling them about to do the test (will mostly happen in other library anyway) or visualization

ModelLoader::ModelLoader() {}

// std::shared_ptr<Assembly> ModelLoader::loadModel(const std::string& filepath) 
// {
//     //Create new assembly
//     std::shared_ptr<Assembly> assembly = std::shared_ptr<Assembly>(new Assembly());

//     //const aiScene* scene = importer_.ReadFile(filepath, aiProcess_Triangulate);
//     const aiScene* scene = importer_.ReadFile(filepath, aiProcess_Triangulate | aiProcess_JoinIdenticalVertices | aiProcess_RemoveRedundantMaterials);


//     if (!scene) {
//         std::cout << "Scene not loaded" << std::endl;
//             return assembly;
//     }

//     std::cout << "Scene loaded" << std::endl;    


//     aiNode* rootNode = scene->mRootNode;


//     std::cout << "Num meshes in scene: " << scene->mNumMeshes << std::endl;

//     for (unsigned int i = 0; i < scene->mNumMeshes; i ++)
//     {
//         std::cout << "Mesh. Num bones: " << scene->mMeshes[i]->mNumBones << " Num faces: " << scene->mMeshes[i]->mNumFaces
//                 << " Has normals: " << scene->mMeshes[i]->HasNormals() << " Has positions: " << scene->mMeshes[i]->HasPositions() << std::endl;

//         float x, y, z = 0;

//         for (unsigned int j = 0; j < scene->mMeshes[i]->mNumVertices; j++)
//         {
//             x += scene->mMeshes[i]->mVertices[j].x;
//             y += scene->mMeshes[i]->mVertices[j].y;
//             z += scene->mMeshes[i]->mVertices[j].z;

//             // std::cout << "Vertex: " << scene->mMeshes[i]->mVertices[j].x << " " 
//             //                         << scene->mMeshes[i]->mVertices[j].y << " "
//             //                         << scene->mMeshes[i]->mVertices[j].z << std::endl;
//         }

//         // for (unsigned int j = 0; j < scene->mMeshes[i]->mNumFaces; j ++)
//         // {
//         //     std::cout << "Face: " << scene->mMeshes[i]->mFaces[j].mIndices[0] << " "  
//         //                             << scene->mMeshes[i]->mFaces[j].mIndices[1] << " "  
//         //                             << scene->mMeshes[i]->mFaces[j].mIndices[2] << std::endl; 

//         // }

//         x = x / scene->mMeshes[i]->mNumVertices;
//         y = y / scene->mMeshes[i]->mNumVertices;
//         z = z / scene->mMeshes[i]->mNumVertices;

//         std::cout << "Average mesh position: " << x << " " << y << " " << z << std::endl;
//     }

//     std::cout << "Name of root node: " << rootNode->mName.C_Str() << std::endl;

//     std::cout << "Num meshes in root node: " << rootNode->mNumMeshes << std::endl;

//     std::cout << "Num children in root node: " << rootNode->mNumChildren << std::endl;

//     aiNode* topNode = rootNode->mChildren[0];

//     std::cout << "Name of top node: " << topNode->mName.C_Str() << std::endl;

//     std::cout << "Num meshes in top node: " << topNode->mNumMeshes << std::endl;

//     std::cout << "Num children in top node: " << topNode->mNumChildren << std::endl;

//     //Get mesh positions

//     std::cout << "Top level node down:" << std::endl;

//     for (unsigned int i = 0; i < topNode->mNumChildren; i ++)
//     {   
//         std::cout << std::endl;

//         aiNode* nameNode = topNode->mChildren[i];


//         std::cout << "Node. Name: " << nameNode->mName.C_Str() << " meshes: " << nameNode->mNumMeshes << " children: " << nameNode->mNumChildren << " Transform: " 
//                     << nameNode->mTransformation.a4 << " " 
//                     << nameNode->mTransformation.b4 << " " 
//                     << nameNode->mTransformation.c4 << std::endl;

//         for (unsigned int j = 0; j < nameNode->mNumChildren; j ++)
//         {
//             aiNode* nameChildNode = nameNode->mChildren[j];

//             std::cout << "Node. Name: " << nameChildNode->mName.C_Str() << " meshes: " << nameChildNode->mNumMeshes << " children: " << nameChildNode->mNumChildren << " Transform: " 
//                     << nameChildNode->mTransformation.a4 << " " 
//                     << nameChildNode->mTransformation.b4 << " " 
//                     << nameChildNode->mTransformation.c4 << std::endl;

//             for (unsigned int k = 0; k < nameChildNode->mNumChildren; k ++)
//             {
//                 aiNode* nameChildChildNode = nameChildNode->mChildren[k];
    
//                 std::cout << "Node. Name: " << nameChildChildNode->mName.C_Str() << " meshes: " << nameChildChildNode->mNumMeshes << " children: " << nameChildChildNode->mNumChildren << " Transform: " 
//                         << nameChildChildNode->mTransformation.a4 << " " 
//                         << nameChildChildNode->mTransformation.b4 << " " 
//                         << nameChildChildNode->mTransformation.c4 << std::endl;
//             }
//         }

//     }


//     //Iterate through each mesh, create a new part including a CGAL mesh and add it to the assembly
//     for (size_t i = 0; i < scene->mNumMeshes; i ++)
//     {
//         std::shared_ptr<Polyhedron> polyhedron = std::shared_ptr<Polyhedron>(new Polyhedron());

//         Part::PART_TYPE type = Part::EXTERNAL;

//         aiMesh* mesh = scene->mMeshes[i];

//         BuildPolyhedron<Polyhedron::HalfedgeDS> builder(mesh);
//         polyhedron->delegate(builder);

        
//         CGAL::Polygon_mesh_processing::remove_isolated_vertices(*polyhedron);
//         //CGAL::Polygon_mesh_processing::stitch_borders(*polyhedron);
//         CGAL::Polygon_mesh_processing::remove_degenerate_faces(*polyhedron);

//         // for (auto v = polyhedron->vertices_begin(); v != polyhedron->vertices_end(); ++v) {
//         //     std::cout << "Vertex: " << v->point() << std::endl;
//         // }
        
//         // for (auto f = polyhedron->facets_begin(); f != polyhedron->facets_end(); ++f) {
//         //     std::cout << "Face: ";
//         //     auto h = f->facet_begin();
//         //     do {
//         //         std::cout << h->vertex()->point() << " ";
//         //         ++h;
//         //     } while (h != f->facet_begin());
//         //     std::cout << std::endl;
//         // }

//         std::cout << "Polyhedron has " << polyhedron->size_of_vertices() << " vertices and "
//               << polyhedron->size_of_facets() << " faces." << std::endl;


//         std::cout << "Is closed: " << polyhedron->is_closed() << std::endl;

//         std::cout << "Self-intersects: " << CGAL::Polygon_mesh_processing::does_self_intersect(*polyhedron) << std::endl;

//         std::shared_ptr<Part> part = std::shared_ptr<Part>(new Part(polyhedron, type, i));

//         assembly->addPart(part);

//         std::cout << "Part centroid: " << part->getCentroid().x() << " " << part->getCentroid().y() << " " << part->getCentroid().z() << std::endl;
//     }

//     return assembly;
// }

// std::string ModelLoader::GetShapeName(const TDF_Label& label) {
//     Handle(TDataStd_Name) nameAttr;
//     if (label.FindAttribute(TDataStd_Name::GetID(), nameAttr)) {
//         TCollection_AsciiString asciiName(nameAttr->Get());
//         return asciiName.ToCString();
//     }
//     return "Unnamed Part";
// }


//std::string ModelLoader::GetShapeName(const TopoDS_Shape& shape, const Handle(XCAFDoc_ShapeTool)& shapeTool) {
    // TDF_Label label;
    // if (!shapeTool->FindShape(shape, label)) {
    //     return "Unnamed Shape";  // Shape not found in document structure
    // }

    // Handle(TDataStd_Name) nameAttr;
    // if (label.FindAttribute(TDataStd_Name::GetID(), nameAttr)) {
    //     return nameAttr->Get().ToExtString();  // Convert ExtendedString to std::string
    // }

    // return "Unnamed Shape";  // No name found
//}



Polyhedron ModelLoader::ConvertToPolyhedron(const TriangleMesh& mesh) {
    Polyhedron P;
    BuildPolyhedron2<Polyhedron::HalfedgeDS> builder(mesh);
    P.delegate(builder);
    return P;
}



TriangleMesh ModelLoader::ExtractMeshFromShape(const TopoDS_Shape& shape) {
    TriangleMesh mesh;
    std::map<gp_Pnt, int, PointComparator> vertexMap;

    std::cout << std::endl << "Extracting mesh" << std::endl;

    for (TopExp_Explorer faceExp(shape, TopAbs_FACE); faceExp.More(); faceExp.Next()) {
        TopoDS_Face face = TopoDS::Face(faceExp.Current());
        TopLoc_Location loc;

        TopAbs_Orientation faceOrientation = face.Orientation();
        bool isReversed = (faceOrientation == TopAbs_REVERSED);

        Handle(Poly_Triangulation) triangulation = BRep_Tool::Triangulation(face, loc);

        if (!triangulation.IsNull()) {

            //Map of vertex indices in face to indices in model
            std::map<int, int> face_to_model_map;

            // Convert OpenCascade vertices to CGAL points
            for (int i = 1; i <= triangulation->NbNodes(); ++i) {
                gp_Pnt p = triangulation->Node(i).Transformed(loc.Transformation());
                CGAL::Point_3<Kernel> cgalPoint(p.X(), p.Y(), p.Z());

                if (vertexMap.find(p) == vertexMap.end()) {
                    vertexMap[p] = mesh.vertices.size();
                    mesh.vertices.push_back(cgalPoint);
                }
                
                face_to_model_map[i] = vertexMap[p];
            }

            // Extract indexed triangle faces
            for (int i = 1; i <= triangulation->NbTriangles(); ++i) {
                Poly_Triangle tri = triangulation->Triangle(i);
                int v1, v2, v3;

                if (isReversed)
                    tri.Get(v1, v3, v2);

                else
                    tri.Get(v1, v2, v3);
                
                //mesh.faces.push_back({v1 - 1, v2 - 1, v3 - 1}); // Convert to 0-based indexing
                mesh.faces.push_back({face_to_model_map[v1], face_to_model_map[v2], face_to_model_map[v3]}); // Convert to 0-based indexing

            }
        }
    }
    return mesh;
}










std::string ModelLoader::GetShapeName(const TDF_Label& label) {
    Handle(TDataStd_Name) nameAttr;
    if (!label.FindAttribute(TDataStd_Name::GetID(), nameAttr)) {
        return "Unnamed Shape";
    }

    // Convert ExtendedString to ASCII
    TCollection_AsciiString asciiName(nameAttr->Get());

    return asciiName.ToCString();
}

std::vector<std::shared_ptr<NamedPolyhedron>> ModelLoader::loadSTEP(const std::string& filename)
{
    std::vector<std::shared_ptr<NamedPolyhedron>> named_meshes;

    // Load STEP file using STEPCAFControl_Reader (handles metadata properly)
    Handle(TDocStd_Document) doc = new TDocStd_Document("STEP");
    STEPCAFControl_Reader stepReader;
    
    if (stepReader.ReadFile(filename.c_str()) != IFSelect_RetDone) {
        std::cerr << "Error: Failed to read STEP file." << std::endl;
        return named_meshes;
    }

    // Transfer to document structure
    stepReader.Transfer(doc);

    // Get the shape tool (manages multiple parts)
    Handle(XCAFDoc_ShapeTool) shapeTool = XCAFDoc_DocumentTool::ShapeTool(doc->Main());
    if (shapeTool.IsNull()) {
        std::cerr << "Error: Unable to retrieve shape tool from document!" << std::endl;
        return named_meshes;
    }

    // Iterate through all components
    TDF_LabelSequence shapeLabels;
    shapeTool->GetShapes(shapeLabels);

    std::cout << "Number of components: " << shapeLabels.Length() << std::endl;

    //TODO - check part names first and find out which ones are parts (they will be tagged)
    int start = 1;

    if (shapeLabels.Length() > 1)
        start = 2;

    for (Standard_Integer i = start; i <= shapeLabels.Length(); ++i) {  //TODO
        TDF_Label label = shapeLabels.Value(i);
        TopoDS_Shape shape = shapeTool->GetShape(label);

        // Get name of the part
        std::string shapeName = GetShapeName(label);
        std::cout << "Component " << i << ": " << shapeName << std::endl;


        // Generate a mesh for the part
        BRepMesh_IncrementalMesh(shape, 0.1);  // Mesh with a 0.1 tolerance

        //     // Extract vertices
        TopExp_Explorer explorer(shape, TopAbs_FACE);
        while (explorer.More()) {
            TopoDS_Face face = TopoDS::Face(explorer.Current());
            explorer.Next();

            TopAbs_Orientation faceOrientation = face.Orientation();
            bool isReversed = (faceOrientation == TopAbs_REVERSED);

            // Get the triangulated mesh from the face
            TopLoc_Location loc;
            Handle(Poly_Triangulation) triangulation = BRep_Tool::Triangulation(face, loc);

            if (!triangulation.IsNull()) {
                std::cout << "  Face has " << triangulation->NbNodes() << " vertices and " << triangulation->NbTriangles() << " triangles." << std::endl;

                if (isReversed)
                    std::cout << "REVERSED" << std::endl;

                for (Standard_Integer j = 1; j <= triangulation->NbNodes(); j++) {
                    gp_Pnt p = triangulation->Node(j);
                    std::cout << "    Vertex: (" << p.X() << ", " << p.Y() << ", " << p.Z() << ")" << std::endl;
                }

                for (int i = 1; i <= triangulation->NbTriangles(); ++i) {
                    Poly_Triangle t = triangulation->Triangle(i);
                    Standard_Integer v1, v2, v3;
                    t.Get(v1, v2, v3);
                    std::cout << "Triangle " << i << ": " << v1 << ", " << v2 << ", " << v3 << std::endl;
                }
            }
        }

        //Set up CGAL mesh
        TriangleMesh mesh = ExtractMeshFromShape(shape);

        //Analyse mesh
        std::cout << std::endl << "Mesh:" << std::endl;
        std::cout << "Num vertices: " << mesh.vertices.size() << std::endl;
        std::cout << "Num faces: " << mesh.faces.size() << std::endl;

        std::cout << "Vertices: " << std::endl;

        for (auto vertex : mesh.vertices)
        {
            std::cout << vertex.x() << " " << vertex.y() << " " << vertex.z() << std::endl;
        }

        std::cout << "Faces: " << std::endl;

        for (auto face : mesh.faces)
        {
            std::cout << face[0] << " " << face[1] << " " << face[2] << std::endl;
        }

        std::shared_ptr<Polyhedron> polyhedron = std::make_shared<Polyhedron>(ConvertToPolyhedron(mesh));

        std::shared_ptr<NamedPolyhedron> named_polyhedron  = std::shared_ptr<NamedPolyhedron>(new NamedPolyhedron());

        named_polyhedron->polyhedron = polyhedron;

        named_polyhedron->name = shapeName;
        
        named_meshes.push_back(named_polyhedron);

        if (!CGAL::is_closed(*polyhedron)) {
            std::cout << "Mesh has open boundaries!" << std::endl;
        } else {
            std::cout << "Mesh is closed!" << std::endl;
        }

        std::vector<Polyhedron::Halfedge_handle> border_edges;
        CGAL::Polygon_mesh_processing::border_halfedges(*polyhedron, std::back_inserter(border_edges));
        std::cout << "Number of open edges: " << border_edges.size() << std::endl;

        // Stitch close edges together
        CGAL::Polygon_mesh_processing::stitch_borders(*polyhedron);

        if (!CGAL::is_closed(*polyhedron)) {
            std::cout << "Mesh has open boundaries!" << std::endl;
        } else {
            std::cout << "Mesh is closed!" << std::endl;
        }

        std::vector<Polyhedron::Halfedge_handle> border_edges2;
        CGAL::Polygon_mesh_processing::border_halfedges(*polyhedron, std::back_inserter(border_edges2));
        std::cout << "Number of open edges: " << border_edges2.size() << std::endl;

        std::cout << "Successfully converted STEP to CGAL Polyhedron with "
              << polyhedron->size_of_vertices() << " vertices and "
              << polyhedron->size_of_facets() << " faces" << std::endl;
    }

    return named_meshes;
}

std::shared_ptr<Assembly> ModelLoader::loadModel(const std::string& filename) {

    //Create new assembly
    std::shared_ptr<Assembly> assembly = std::shared_ptr<Assembly>(new Assembly());

    std::vector<std::shared_ptr<NamedPolyhedron>> named_polyhedrons = loadSTEP(filename);
    
    int id = 0;

    for (std::shared_ptr<NamedPolyhedron> named_polyhedron : named_polyhedrons)
    {
        Part::PART_TYPE type = Part::EXTERNAL;  //TODO

        std::shared_ptr<Part> part = std::shared_ptr<Part>(new Part(named_polyhedron->polyhedron, type, id));

        assembly->addPart(part);

        id ++;
    }      

    return assembly;
}


std::shared_ptr<Substrate> ModelLoader::loadSubstrate(const std::string& filename)
{
    std::cout << "Loading substrate" << std::endl;

    std::shared_ptr<Substrate> substrate = std::shared_ptr<Substrate>(new Substrate());

    std::vector<std::shared_ptr<NamedPolyhedron>> named_polyhedrons = loadSTEP(filename);

    for (auto named_polyhedron : named_polyhedrons)
    {
        std::cout << "Substrate polyhedron: " << named_polyhedron->name << std::endl;
    }


    if (named_polyhedrons.size() != 1)
    {
        std::cout << "Wrong number of polyhedrons in substrate" << std::endl;
        return substrate;
    }

    substrate->setMesh(named_polyhedrons[0]->polyhedron);

    return substrate;
}