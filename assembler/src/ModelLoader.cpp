#include "assembler/ModelLoader.hpp"
#include "assembler/Assembly.hpp"

#include <iostream>
#include <algorithm>
#include <cctype>
#include <string>



TriangleMesh ModelLoader::ExtractMeshFromShape(const TopoDS_Shape& shape, const gp_Trsf& partTransform, int shape_index) {
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


            std::cout << "Location transformation for part: " << loc.Transformation().TranslationPart().X()
               << ", " << loc.Transformation().TranslationPart().Y()
             << ", " << loc.Transformation().TranslationPart().Z() << std::endl;

            // **Compute final transformation: part + local face transformation**
            gp_Trsf finalTrsf = partTransform * loc.Transformation();

            // Map of vertex indices in face to indices in model
            std::map<int, int> face_to_model_map;

            // Convert OpenCascade vertices to CGAL points
            for (int i = 1; i <= triangulation->NbNodes(); ++i) {
                gp_Pnt p = triangulation->Node(i).Transformed(finalTrsf);  // **Apply assembly transform**

                //High level node
                if (shape_index == 1)
                {
                    points_.push_back(p.X());
                    points_.push_back(p.Y());
                    points_.push_back(p.Z());
                
                    std::cout << "Point: " << p.X() << " " << p.Y() << " " << p.Z() << std::endl;

                    CGAL::Point_3<Kernel> cgalPoint(p.X(), p.Y(), p.Z());

                    if (vertexMap.find(p) == vertexMap.end()) {
                        vertexMap[p] = mesh.vertices.size();
                        mesh.vertices.push_back(cgalPoint);
                    }

                    face_to_model_map[i] = vertexMap[p];
                }

                //Subnodes
                else
                {
                    CGAL::Point_3<Kernel> cgalPoint(points_[points_index_], points_[points_index_ + 1], points_[points_index_ + 2]);

                    std::cout << "Point: " << points_[points_index_] << " " << points_[points_index_ + 1] << " " << points_[points_index_ + 2] << std::endl;

                    if (vertexMap.find(p) == vertexMap.end()) {
                        vertexMap[p] = mesh.vertices.size();
                        mesh.vertices.push_back(cgalPoint);
                    }

                    face_to_model_map[i] = vertexMap[p];

                    points_index_ += 3;
                }
            }

            // Extract indexed triangle faces
            for (int i = 1; i <= triangulation->NbTriangles(); ++i) {
                Poly_Triangle tri = triangulation->Triangle(i);
                int v1, v2, v3;

                if (isReversed)
                    tri.Get(v1, v3, v2);
                else
                    tri.Get(v1, v2, v3);

                mesh.faces.push_back({face_to_model_map[v1], face_to_model_map[v2], face_to_model_map[v3]});
            }
        }
    }
    return mesh;
}

// TriangleMesh ModelLoader::ExtractMeshFromShape(const TopoDS_Shape& shape) {
//     TriangleMesh mesh;
//     std::map<gp_Pnt, int, PointComparator> vertexMap;

//     std::cout << std::endl << "Extracting mesh" << std::endl;

//     // Get the transformation of the entire shape
//     TopLoc_Location globalLoc = shape.Location();
//     gp_Trsf globalTrsf = globalLoc.Transformation();


//     std::cout << "Global transformation for part: " << globalTrsf.TranslationPart().X()
//           << ", " << globalTrsf.TranslationPart().Y()
//           << ", " << globalTrsf.TranslationPart().Z() << std::endl;

//     for (TopExp_Explorer faceExp(shape, TopAbs_FACE); faceExp.More(); faceExp.Next()) {
//         TopoDS_Face face = TopoDS::Face(faceExp.Current());
//         TopLoc_Location loc;

//         TopAbs_Orientation faceOrientation = face.Orientation();
//         bool isReversed = (faceOrientation == TopAbs_REVERSED);

//         // Get triangulation and local transformation
//         Handle(Poly_Triangulation) triangulation = BRep_Tool::Triangulation(face, loc);

//         if (!triangulation.IsNull()) {

//             std::cout << "Location transformation for part: " << loc.Transformation().TranslationPart().X()
//           << ", " << loc.Transformation().TranslationPart().Y()
//           << ", " << loc.Transformation().TranslationPart().Z() << std::endl;


//             // Compute full transformation: global + local
//             gp_Trsf finalTrsf = globalTrsf * loc.Transformation();

//             // Map of vertex indices in face to indices in model
//             std::map<int, int> face_to_model_map;

//             // Convert OpenCascade vertices to CGAL points
//             for (int i = 1; i <= triangulation->NbNodes(); ++i) {
//                 gp_Pnt p = triangulation->Node(i).Transformed(finalTrsf);

//                 std::cout << "Point: " << p.X() << " " << p.Y() << " " << p.Z() << std::endl;

//                 CGAL::Point_3<Kernel> cgalPoint(p.X(), p.Y(), p.Z());

//                 if (vertexMap.find(p) == vertexMap.end()) {
//                     vertexMap[p] = mesh.vertices.size();
//                     mesh.vertices.push_back(cgalPoint);
//                 }

//                 face_to_model_map[i] = vertexMap[p];
//             }

//             // Extract indexed triangle faces
//             for (int i = 1; i <= triangulation->NbTriangles(); ++i) {
//                 Poly_Triangle tri = triangulation->Triangle(i);
//                 int v1, v2, v3;

//                 if (isReversed)
//                     tri.Get(v1, v3, v2);
//                 else
//                     tri.Get(v1, v2, v3);

//                 mesh.faces.push_back({face_to_model_map[v1], face_to_model_map[v2], face_to_model_map[v3]});
//             }
//         }
//     }
//     return mesh;
// }

//original
// TriangleMesh ModelLoader::ExtractMeshFromShape(const TopoDS_Shape& shape) {
//     TriangleMesh mesh;
//     std::map<gp_Pnt, int, PointComparator> vertexMap;

//     std::cout << std::endl << "Extracting mesh" << std::endl;

//     for (TopExp_Explorer faceExp(shape, TopAbs_FACE); faceExp.More(); faceExp.Next()) {
//         TopoDS_Face face = TopoDS::Face(faceExp.Current());
//         TopLoc_Location loc;

//         TopAbs_Orientation faceOrientation = face.Orientation();
//         bool isReversed = (faceOrientation == TopAbs_REVERSED);

//         Handle(Poly_Triangulation) triangulation = BRep_Tool::Triangulation(face, loc);

//         if (!triangulation.IsNull()) {

//             //Map of vertex indices in face to indices in model
//             std::map<int, int> face_to_model_map;


//             std::cout << "Location transformation for part: " << loc.Transformation().TranslationPart().X()
//                        << ", " << loc.Transformation().TranslationPart().Y()
//                        << ", " << loc.Transformation().TranslationPart().Z() << std::endl;

//             // Convert OpenCascade vertices to CGAL points
//             for (int i = 1; i <= triangulation->NbNodes(); ++i) {
//                 gp_Pnt p = triangulation->Node(i).Transformed(loc.Transformation());
//                 gp_Pnt q = triangulation->Node(i);

//                 std::cout << "Transformed Point: " << p.X() << " " << p.Y() << " " << p.Z() << std::endl;

//                 std::cout << "Untransformed Point: " << q.X() << " " << q.Y() << " " << q.Z() << std::endl;
                
//                 //gp_Pnt p = triangulation->Node(i);
//                 CGAL::Point_3<Kernel> cgalPoint(p.X(), p.Y(), p.Z());

//                 if (vertexMap.find(p) == vertexMap.end()) {
//                     vertexMap[p] = mesh.vertices.size();
//                     mesh.vertices.push_back(cgalPoint);
//                 }
                
//                 face_to_model_map[i] = vertexMap[p];
//             }

//             // Extract indexed triangle faces
//             for (int i = 1; i <= triangulation->NbTriangles(); ++i) {
//                 Poly_Triangle tri = triangulation->Triangle(i);
//                 int v1, v2, v3;

//                 if (isReversed)
//                     tri.Get(v1, v3, v2);

//                 else
//                     tri.Get(v1, v2, v3);
                
//                 //mesh.faces.push_back({v1 - 1, v2 - 1, v3 - 1}); // Convert to 0-based indexing
//                 mesh.faces.push_back({face_to_model_map[v1], face_to_model_map[v2], face_to_model_map[v3]}); // Convert to 0-based indexing

//             }
//         }
//     }
//     return mesh;
// }










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
    points_.clear();

    points_index_ = 0;


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

    for (Standard_Integer i = 1; i <= shapeLabels.Length(); ++i) {  //TODO


        TDF_Label label = shapeLabels.Value(i);
        TopoDS_Shape shape = shapeTool->GetShape(label);

        // // Get name of the part
        std::string shapeName = GetShapeName(label);
        std::cout << "Component " << i << ": " << shapeName << std::endl;

        // Get the instance transformation **from the assembly**
        TopLoc_Location instanceLocation = shape.Location();
        gp_Trsf instanceTransform = instanceLocation.Transformation();

        std::cout << "Instance transformation: "
                << instanceTransform.TranslationPart().X() << ", "
                << instanceTransform.TranslationPart().Y() << ", "
                << instanceTransform.TranslationPart().Z() << std::endl;

        BRepMesh_IncrementalMesh(shape, 0.1);  // Mesh with a 0.1 tolerance

        // Extract mesh with the correct instance transform
        TriangleMesh mesh = ExtractMeshFromShape(shape, instanceTransform, i);



        // TDF_Label label = shapeLabels.Value(i);
        // TopoDS_Shape shape = shapeTool->GetShape(label);

        // // Get name of the part
        // std::string shapeName = GetShapeName(label);
        // std::cout << "Component " << i << ": " << shapeName << std::endl;


        ///////

        //BRepMesh_IncrementalMesh(shape, 0.1);  // Mesh with a 0.1 tolerance


        // // **Get the transformation of this part in the assembly**
        // TopLoc_Location partLocation = shape.Location();
        // gp_Trsf partTransform = partLocation.Transformation();

        // std::cout << "Part transformation: " 
        //         << partTransform.TranslationPart().X() << ", "
        //         << partTransform.TranslationPart().Y() << ", "
        //         << partTransform.TranslationPart().Z() << std::endl;

        // // Generate a mesh for the part, applying part-level transformation
        // TriangleMesh mesh = ExtractMeshFromShape(shape, partTransform);

        ////////////

        // // Generate a mesh for the part
        // BRepMesh_IncrementalMesh(shape, 0.1);  // Mesh with a 0.1 tolerance

        // //Set up CGAL mesh
        // TriangleMesh mesh = ExtractMeshFromShape(shape);

        //Analyse mesh
        std::cout << std::endl << "Mesh:" << std::endl;
        std::cout << "Num vertices: " << mesh.vertices.size() << std::endl;
        std::cout << "Num faces: " << mesh.faces.size() << std::endl;


        Polyhedron P;
        BuildPolyhedron<Polyhedron::HalfedgeDS> builder(mesh);
        P.delegate(builder);

        std::shared_ptr<Polyhedron> polyhedron = std::make_shared<Polyhedron>(P);


        std::cout << "Loaded mesh " << shapeName << " at " << meshCenter(polyhedron).x() << " " << meshCenter(polyhedron).y() << " " << meshCenter(polyhedron).z() << std::endl; 

        std::cout << "X span: " << meshBoundingBox(polyhedron).xmin() << " " << meshBoundingBox(polyhedron).xmax() << std::endl;
        std::cout << "Y span: " << meshBoundingBox(polyhedron).ymin() << " " << meshBoundingBox(polyhedron).ymax() << std::endl;
        std::cout << "Z span: " << meshBoundingBox(polyhedron).zmin() << " " << meshBoundingBox(polyhedron).zmax() << std::endl;


        std::shared_ptr<NamedPolyhedron> named_polyhedron  = std::shared_ptr<NamedPolyhedron>(new NamedPolyhedron());

        named_polyhedron->polyhedron = polyhedron;


        //Set shape name to lower case
        std::transform(shapeName.begin(), shapeName.end(), shapeName.begin(), [](unsigned char c){ return std::tolower(c); });

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
        //CGAL::Polygon_mesh_processing::remove_self_intersections(*polyhedron);
        CGAL::Polygon_mesh_processing::stitch_borders(*polyhedron);
        //CGAL::Polygon_mesh_processing::orient_polygon_soup(*polyhedron); 

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
        Part::PART_TYPE type;

        if (named_polyhedron->name.find("internal") != std::string::npos)
            type = Part::INTERNAL;

        else if (named_polyhedron->name.find("external") != std::string::npos)
            type = Part::EXTERNAL;

        else if (named_polyhedron->name.find("screw") != std::string::npos)
            type = Part::SCREW;

        else
            continue;

        std::shared_ptr<Part> part = std::shared_ptr<Part>(new Part(named_polyhedron->polyhedron, type, id, named_polyhedron->name));

        assembly->addPart(part);

        id ++;
    }      

    return assembly;
}


std::shared_ptr<MeshObject> ModelLoader::loadSubstrate(const std::string& filename)
{
    std::cout << "Loading substrate" << std::endl;

    std::shared_ptr<MeshObject> substrate = std::shared_ptr<MeshObject>(new MeshObject());

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