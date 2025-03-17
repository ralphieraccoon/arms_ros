#include "assembler/ModelLoader.hpp"
#include "assembler/Assembly.hpp"

#include <iostream>
#include <algorithm>
#include <cctype>
#include <string>

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
    //int start = 1;

    //if (shapeLabels.Length() > 1)
        //start = 2;

    for (Standard_Integer i = 1; i <= shapeLabels.Length(); ++i) {  //TODO
        TDF_Label label = shapeLabels.Value(i);
        TopoDS_Shape shape = shapeTool->GetShape(label);

        // Get name of the part
        std::string shapeName = GetShapeName(label);
        std::cout << "Component " << i << ": " << shapeName << std::endl;


        // Generate a mesh for the part
        BRepMesh_IncrementalMesh(shape, 0.1);  // Mesh with a 0.1 tolerance

        // Extract vertices
        TopExp_Explorer explorer(shape, TopAbs_FACE);
        while (explorer.More()) {
            TopoDS_Face face = TopoDS::Face(explorer.Current());
            explorer.Next();

            TopAbs_Orientation faceOrientation = face.Orientation();
            bool isReversed = (faceOrientation == TopAbs_REVERSED);

            // Get the triangulated mesh from the face
            TopLoc_Location loc;
            Handle(Poly_Triangulation) triangulation = BRep_Tool::Triangulation(face, loc);

            // if (!triangulation.IsNull()) {
            //     std::cout << "  Face has " << triangulation->NbNodes() << " vertices and " << triangulation->NbTriangles() << " triangles." << std::endl;

            //     if (isReversed)
            //         std::cout << "REVERSED" << std::endl;

            //     for (Standard_Integer j = 1; j <= triangulation->NbNodes(); j++) {
            //         gp_Pnt p = triangulation->Node(j);
            //         std::cout << "    Vertex: (" << p.X() << ", " << p.Y() << ", " << p.Z() << ")" << std::endl;
            //     }

            //     for (int i = 1; i <= triangulation->NbTriangles(); ++i) {
            //         Poly_Triangle t = triangulation->Triangle(i);
            //         Standard_Integer v1, v2, v3;
            //         t.Get(v1, v2, v3);
            //         std::cout << "Triangle " << i << ": " << v1 << ", " << v2 << ", " << v3 << std::endl;
            //     }
            // }
        }

        //Set up CGAL mesh
        TriangleMesh mesh = ExtractMeshFromShape(shape);

        //Analyse mesh
        std::cout << std::endl << "Mesh:" << std::endl;
        std::cout << "Num vertices: " << mesh.vertices.size() << std::endl;
        std::cout << "Num faces: " << mesh.faces.size() << std::endl;

        // std::cout << "Vertices: " << std::endl;

        // for (auto vertex : mesh.vertices)
        // {
        //     std::cout << vertex.x() << " " << vertex.y() << " " << vertex.z() << std::endl;
        // }

        // std::cout << "Faces: " << std::endl;

        // for (auto face : mesh.faces)
        // {
        //     std::cout << face[0] << " " << face[1] << " " << face[2] << std::endl;
        // }


        Polyhedron P;
        BuildPolyhedron<Polyhedron::HalfedgeDS> builder(mesh);
        P.delegate(builder);

        std::shared_ptr<Polyhedron> polyhedron = std::make_shared<Polyhedron>(P);

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