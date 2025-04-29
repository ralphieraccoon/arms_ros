#include "assembler/ModelLoader.hpp"
#include "assembler/Assembly.hpp"


std::shared_ptr<Assembly> ModelLoader::loadModel(const std::string& filename) {

    //Create new assembly
    std::shared_ptr<Assembly> assembly = std::shared_ptr<Assembly>(new Assembly());

    std::vector<std::shared_ptr<Part>> parts = loadSTEP(filename);

    for (std::shared_ptr<Part> part : parts)
        assembly->addPart(part);

    assembly->saveAsSTL("target_assembly.stl");

    return assembly;
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

std::vector<std::shared_ptr<Part>> ModelLoader::loadSTEP(const std::string& filename)
{
    std::vector<std::shared_ptr<Part>> parts;

    // Load STEP file using STEPCAFControl_Reader (handles metadata properly)
    Handle(TDocStd_Document) doc = new TDocStd_Document("STEP");
    STEPCAFControl_Reader stepReader;

    if (stepReader.ReadFile(filename.c_str()) != IFSelect_RetDone) {
        std::cerr << "Error: Failed to read STEP file." << std::endl;
        return parts;
    }

    // Transfer to document structure
    stepReader.Transfer(doc);
    
    // Get the shape tool (manages multiple parts)
    Handle(XCAFDoc_ShapeTool) shapeTool = XCAFDoc_DocumentTool::ShapeTool(doc->Main());
    if (shapeTool.IsNull()) {
        std::cerr << "Error: Unable to retrieve shape tool from document!" << std::endl;
        return parts;
    }

    TDF_LabelSequence freeShapes;

    shapeTool->GetFreeShapes(freeShapes);  // top-level shapes (assemblies or parts)

    if (freeShapes.Length() != 1)
        std::cerr << "Incorrect number of free shapes" << std::endl;

    TDF_Label assemblyLabel = freeShapes.Value(1);

    RecurrentAddPart(assemblyLabel, shapeTool, parts, 0);

    return parts;
}

void ModelLoader::RecurrentAddPart(const TDF_Label& label, const Handle(XCAFDoc_ShapeTool)& shapeTool, std::vector<std::shared_ptr<Part>>& parts, int level)
{
    std::string name = GetShapeName(label);

    Part::PART_TYPE type;

    if (name.find("internal") != std::string::npos)
        type = Part::INTERNAL;

    else if (name.find("external") != std::string::npos)
        type = Part::EXTERNAL;

    else if (name.find("screw") != std::string::npos)
        type = Part::SCREW;

    else
        type = Part::NONE;

    if (type != Part::NONE)
        parts.push_back(std::shared_ptr<Part>(new Part(std::make_shared<TopoDS_Shape>(shapeTool->GetShape(label)), type, next_id_++, name)));

    std::cout << level << " " << name << std::endl;

    TDF_LabelSequence children;

    shapeTool->GetComponents(label, children);

    for (Standard_Integer i = 1; i <= children.Length(); ++i)
    {
        RecurrentAddPart(children.Value(i), shapeTool, parts, level + 1);
    }
}





















































// //TODODOOOODOODOD
// int part_index = 0;

// bool ShapesIntersect(const TopoDS_Shape& shape1, const TopoDS_Shape& shape2, double threshold = 1e-6) {
//     BRepExtrema_DistShapeShape distCalc(shape1, shape2);
//     if (distCalc.IsDone()) {

//         std::cout << "Intersection: " << distCalc.Value() << std::endl;

//         if (distCalc.Value() < 0)
//             return true;
        
//         else
//             return false;
//     }
//     return false;  // If the distance couldn't be computed
// }

// void PrintFaceInfo(const TopoDS_Face& face) {
//     // Orientation
//     TopAbs_Orientation orient = face.Orientation();
//     std::cout << "Orientation: ";
//     switch (orient) {
//         case TopAbs_FORWARD:  std::cout << "FORWARD"; break;
//         case TopAbs_REVERSED: std::cout << "REVERSED"; break;
//         case TopAbs_INTERNAL: std::cout << "INTERNAL"; break;
//         case TopAbs_EXTERNAL: std::cout << "EXTERNAL"; break;
//         default:              std::cout << "UNKNOWN"; break;
//     }
//     std::cout << std::endl;

//     // Surface and UV midpoint
//     Handle(Geom_Surface) surf = BRep_Tool::Surface(face);
//     BRepAdaptor_Surface adaptor(face);
//     Standard_Real uMid = 0.5 * (adaptor.FirstUParameter() + adaptor.LastUParameter());
//     Standard_Real vMid = 0.5 * (adaptor.FirstVParameter() + adaptor.LastVParameter());

//     // Point on surface
//     gp_Pnt p = adaptor.Value(uMid, vMid);
//     std::cout << "Approx Center Point: (" << p.X() << ", " << p.Y() << ", " << p.Z() << ")" << std::endl;

//     // Surface normal
//     GeomLProp_SLProps props(surf, uMid, vMid, 1, Precision::Confusion());
//     if (props.IsNormalDefined()) {
//         gp_Vec n = props.Normal();
//         std::cout << "Normal at Center:    (" << n.X() << ", " << n.Y() << ", " << n.Z() << ")" << std::endl;
//     } else {
//         std::cout << "Normal is not defined at this point." << std::endl;
//     }

//     // Bounding box
//     Bnd_Box bbox;
//     BRepBndLib::Add(face, bbox);
//     Standard_Real xmin, ymin, zmin, xmax, ymax, zmax;
//     bbox.Get(xmin, ymin, zmin, xmax, ymax, zmax);
//     std::cout << "Bounding Box:\n"
//               << "  X: [" << xmin << ", " << xmax << "]\n"
//               << "  Y: [" << ymin << ", " << ymax << "]\n"
//               << "  Z: [" << zmin << ", " << zmax << "]" << std::endl;

//     // Surface type
//     std::cout << "Surface Type: " << surf->DynamicType()->Name() << std::endl;
// }

// // Optional: convert shape type to string for debug
// std::string ShapeTypeToString(TopAbs_ShapeEnum type) {
//     switch (type) {
//         case TopAbs_COMPOUND: return "COMPOUND";
//         case TopAbs_COMPSOLID: return "COMPSOLID";
//         case TopAbs_SOLID: return "SOLID";
//         case TopAbs_SHELL: return "SHELL";
//         case TopAbs_FACE: return "FACE";
//         case TopAbs_EDGE: return "EDGE";
//         case TopAbs_VERTEX: return "VERTEX";
//         default: return "UNKNOWN";
//     }
// }

// void TraverseAndPrintShapes(
//     const Handle(XCAFDoc_ShapeTool)& shapeTool,
//     const TDF_Label& label,
//     const gp_Trsf& parentTrsf,
//     int depth = 0
// ) {
//     // Indentation for pretty output
//     std::string indent(depth * 2, ' ');

//     if (!shapeTool->IsShape(label)) return;

//     TopoDS_Shape shape = shapeTool->GetShape(label);
//     TopLoc_Location loc = shape.Location();
//     gp_Trsf localTrsf = loc.Transformation();

//     // Combine with parent transformation
//     gp_Trsf combinedTrsf = parentTrsf.Multiplied(localTrsf);

//     // Optional: apply to shape for visualization/processing
//     TopoDS_Shape transformedShape = shape.Moved(TopLoc_Location(combinedTrsf));

//     std::cout << indent << "Shape Type: " << ShapeTypeToString(shape.ShapeType()) << "\n";
//     std::cout << indent << "  Position: "
//               << combinedTrsf.TranslationPart().X() << ", "
//               << combinedTrsf.TranslationPart().Y() << ", "
//               << combinedTrsf.TranslationPart().Z() << "\n";

//     // Optional: explore solids inside this shape
//     for (TopExp_Explorer exp(shape, TopAbs_SOLID); exp.More(); exp.Next()) {
//         TopoDS_Solid solid = TopoDS::Solid(exp.Current());
//         gp_Trsf solidTrsf = solid.Location().Transformation();

        

//         std::cout << indent << "  ↳ Solid Position: "
//                   << solidTrsf.TranslationPart().X() << ", "
//                   << solidTrsf.TranslationPart().Y() << ", "
//                   << solidTrsf.TranslationPart().Z() << "\n";
//     }

//     // Optional: explore solids inside this shape
//     for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next()) {
//         TopoDS_Face solid = TopoDS::Face(exp.Current());
//         gp_Trsf solidTrsf = solid.Location().Transformation();

//         PrintFaceInfo(solid);

//         std::cout << indent << "  ↳ Face Position: "
//                   << solidTrsf.TranslationPart().X() << ", "
//                   << solidTrsf.TranslationPart().Y() << ", "
//                   << solidTrsf.TranslationPart().Z() << "\n";
//     }

//     // Optional: explore solids inside this shape
//     for (TopExp_Explorer exp(shape, TopAbs_VERTEX); exp.More(); exp.Next()) {
//         TopoDS_Vertex solid = TopoDS::Vertex(exp.Current());
        
//         gp_Trsf solidTrsf = solid.Location().Transformation();

//         // gp_Pnt point = BRep_Tool::Pnt(solid);
//         // std::cout << "Vertex at: "
//         //   << point.X() << ", "
//         //   << point.Y() << ", "
//         //   << point.Z() << std::endl;

        

//         std::cout << indent << "  ↳ Vertex Position: "
//                   << solidTrsf.TranslationPart().X() << ", "
//                   << solidTrsf.TranslationPart().Y() << ", "
//                   << solidTrsf.TranslationPart().Z() << "\n";
//     }

//     // Recurse into child labels
//     for (TDF_ChildIterator it(label); it.More(); it.Next()) {
//         TraverseAndPrintShapes(shapeTool, it.Value(), combinedTrsf, depth + 1);
//     }
// }

// void TraverseAssembly(const Handle(XCAFDoc_ShapeTool)& shapeTool, const TDF_Label& label, const gp_Trsf& parentTrsf, std::vector<std::shared_ptr<Part>> &parts) {
//     TopoDS_Shape shape = shapeTool->GetShape(label);
//     TopLoc_Location loc = shape.Location();
//     gp_Trsf trsf = loc.Transformation();
//     gp_Trsf combinedTrsf = parentTrsf.Multiplied(trsf);

//     // Get name (if needed)


//     Handle(TDataStd_Name) nameAttr;
//     if (!label.FindAttribute(TDataStd_Name::GetID(), nameAttr)) {
//         std::cout << "Unnamed Shape" << std::endl;
//     }

//     // Convert ExtendedString to ASCII
//     TCollection_AsciiString asciiName(nameAttr->Get());

//     std::string name = asciiName.ToCString();

//     // Handle(TDataStd_Name) nameAttr;
//     // if (label.FindAttribute(TDataStd_Name::GetID(), nameAttr)) {
//     //     std::cout << "Name: " << nameAttr->Get() << std::endl;
//     // }

//     std::cout << "Name: " << name << std::endl;

//     // extStr is a TCollection_ExtendedString
//     //Standard_ExtString extStr = nameAttr->Get().ToExtString();
//     //std::wstring wstr(extStr);  // Convert to std::wstring

//     // Then convert wstring to string (naive narrow cast)
//     //std::string name(wstr.begin(), wstr.end());

//     //std::string name = std::string(nameAttr->Get().ToExtString());

//     // Do something with shape.Located(TopLoc_Location(combinedTrsf))
//     // e.g., store it or print its position
//     gp_XYZ pos = combinedTrsf.TranslationPart();
//     std::cout << "  Position: " << pos.X() << ", " << pos.Y() << ", " << pos.Z() << std::endl;


//     gp_Mat rotation = combinedTrsf.HVectorialPart();
//     std::cout << "  Rotation matrix:" << std::endl;
//     for (int i = 1; i <= 3; ++i) {
//         std::cout << "    ";
//         for (int j = 1; j <= 3; ++j) {
//             std::cout << rotation.Value(i, j) << " ";
//         }
//         std::cout << std::endl;
//     }


//     // Apply the location (transformation) to the shape


//     TopoDS_Shape transformedShape = shape.Moved(TopLoc_Location(combinedTrsf));


//     BRepBuilderAPI_Copy copier(transformedShape);
//     TopoDS_Shape copiedShape = copier.Shape();

//     Part::PART_TYPE type;

//     if (name.find("internal") != std::string::npos)
//         type = Part::INTERNAL;

//     else if (name.find("external") != std::string::npos)
//         type = Part::EXTERNAL;

//     else if (name.find("screw") != std::string::npos)
//         type = Part::SCREW;

//     else
//         type = Part::NONE; //TODO: NONE

//     if (type != Part::NONE)
//     {
//         parts.push_back(std::shared_ptr<Part>(new Part(std::make_shared<TopoDS_Shape>(copiedShape), type, part_index, name)));

//         part_index ++;
//     }
//     // Recurse into children
//     TDF_LabelSequence children;
//     shapeTool->GetComponents(label, children);
//     for (Standard_Integer i = 1; i <= children.Length(); ++i) {
//         TraverseAssembly(shapeTool, children.Value(i), combinedTrsf, parts);
//     }
// }

// void PrintLabelTree(const TDF_Label& label, int depth = 0) {
//     // Indent
//     for (int i = 0; i < depth; ++i) std::cout << "  ";

//     // Print label tag
//     std::cout << "Label: " << label.Tag();

//     // Print name if exists
//     Handle(TDataStd_Name) nameAttr;
//     if (label.FindAttribute(TDataStd_Name::GetID(), nameAttr)) {
//         std::cout << " (" << nameAttr->Get() << ")";
//     }

//     // Print shape info if it has one
//     if (XCAFDoc_ShapeTool::IsShape(label)) {
//         std::cout << " [Shape]";
//         TopoDS_Shape shape = XCAFDoc_ShapeTool::GetShape(label);
//         std::cout << " - Type: " << ShapeTypeToString(shape.ShapeType());
//     }

//     std::cout << std::endl;

//     // Recurse into children
//     for (TDF_ChildIterator it(label); it.More(); it.Next()) {
//         PrintLabelTree(it.Value(), depth + 1);
//     }
// }

// TriangleMesh ModelLoader::ExtractMeshFromShape(const TopoDS_Shape& shape, const gp_Trsf& partTransform, int shape_index) {
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


//             //std::cout << "Location transformation for part: " << loc.Transformation().TranslationPart().X()
//             //   << ", " << loc.Transformation().TranslationPart().Y()
//             // << ", " << loc.Transformation().TranslationPart().Z() << std::endl;

//             // **Compute final transformation: part + local face transformation**
//             gp_Trsf finalTrsf = partTransform * loc.Transformation();

//             // Map of vertex indices in face to indices in model
//             std::map<int, int> face_to_model_map;

//             // Convert OpenCascade vertices to CGAL points
//             for (int i = 1; i <= triangulation->NbNodes(); ++i) {
//                 gp_Pnt p = triangulation->Node(i).Transformed(finalTrsf);  // **Apply assembly transform**

//                 //High level node
//                 if (shape_index == 1)
//                 {
//                     points_.push_back(p.X());
//                     points_.push_back(p.Y());
//                     points_.push_back(p.Z());
                
//                     //std::cout << "Point: " << p.X() << " " << p.Y() << " " << p.Z() << std::endl;

//                     CGAL::Point_3<Kernel> cgalPoint(p.X(), p.Y(), p.Z());

//                     if (vertexMap.find(p) == vertexMap.end()) {
//                         vertexMap[p] = mesh.vertices.size();
//                         mesh.vertices.push_back(cgalPoint);
//                     }

//                     face_to_model_map[i] = vertexMap[p];
//                 }

//                 //Subnodes
//                 else
//                 {
//                     CGAL::Point_3<Kernel> cgalPoint(points_[points_index_], points_[points_index_ + 1], points_[points_index_ + 2]);

//                     //std::cout << "Point: " << points_[points_index_] << " " << points_[points_index_ + 1] << " " << points_[points_index_ + 2] << std::endl;

//                     if (vertexMap.find(p) == vertexMap.end()) {
//                         vertexMap[p] = mesh.vertices.size();
//                         mesh.vertices.push_back(cgalPoint);
//                     }

//                     face_to_model_map[i] = vertexMap[p];

//                     points_index_ += 3;
//                 }
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

// // TriangleMesh ModelLoader::ExtractMeshFromShape(const TopoDS_Shape& shape) {
// //     TriangleMesh mesh;
// //     std::map<gp_Pnt, int, PointComparator> vertexMap;

// //     std::cout << std::endl << "Extracting mesh" << std::endl;

// //     // Get the transformation of the entire shape
// //     TopLoc_Location globalLoc = shape.Location();
// //     gp_Trsf globalTrsf = globalLoc.Transformation();


// //     std::cout << "Global transformation for part: " << globalTrsf.TranslationPart().X()
// //           << ", " << globalTrsf.TranslationPart().Y()
// //           << ", " << globalTrsf.TranslationPart().Z() << std::endl;

// //     for (TopExp_Explorer faceExp(shape, TopAbs_FACE); faceExp.More(); faceExp.Next()) {
// //         TopoDS_Face face = TopoDS::Face(faceExp.Current());
// //         TopLoc_Location loc;

// //         TopAbs_Orientation faceOrientation = face.Orientation();
// //         bool isReversed = (faceOrientation == TopAbs_REVERSED);

// //         // Get triangulation and local transformation
// //         Handle(Poly_Triangulation) triangulation = BRep_Tool::Triangulation(face, loc);

// //         if (!triangulation.IsNull()) {

// //             std::cout << "Location transformation for part: " << loc.Transformation().TranslationPart().X()
// //           << ", " << loc.Transformation().TranslationPart().Y()
// //           << ", " << loc.Transformation().TranslationPart().Z() << std::endl;


// //             // Compute full transformation: global + local
// //             gp_Trsf finalTrsf = globalTrsf * loc.Transformation();

// //             // Map of vertex indices in face to indices in model
// //             std::map<int, int> face_to_model_map;

// //             // Convert OpenCascade vertices to CGAL points
// //             for (int i = 1; i <= triangulation->NbNodes(); ++i) {
// //                 gp_Pnt p = triangulation->Node(i).Transformed(finalTrsf);

// //                 std::cout << "Point: " << p.X() << " " << p.Y() << " " << p.Z() << std::endl;

// //                 CGAL::Point_3<Kernel> cgalPoint(p.X(), p.Y(), p.Z());

// //                 if (vertexMap.find(p) == vertexMap.end()) {
// //                     vertexMap[p] = mesh.vertices.size();
// //                     mesh.vertices.push_back(cgalPoint);
// //                 }

// //                 face_to_model_map[i] = vertexMap[p];
// //             }

// //             // Extract indexed triangle faces
// //             for (int i = 1; i <= triangulation->NbTriangles(); ++i) {
// //                 Poly_Triangle tri = triangulation->Triangle(i);
// //                 int v1, v2, v3;

// //                 if (isReversed)
// //                     tri.Get(v1, v3, v2);
// //                 else
// //                     tri.Get(v1, v2, v3);

// //                 mesh.faces.push_back({face_to_model_map[v1], face_to_model_map[v2], face_to_model_map[v3]});
// //             }
// //         }
// //     }
// //     return mesh;
// // }

// //original
// // TriangleMesh ModelLoader::ExtractMeshFromShape(const TopoDS_Shape& shape) {
// //     TriangleMesh mesh;
// //     std::map<gp_Pnt, int, PointComparator> vertexMap;

// //     std::cout << std::endl << "Extracting mesh" << std::endl;

// //     for (TopExp_Explorer faceExp(shape, TopAbs_FACE); faceExp.More(); faceExp.Next()) {
// //         TopoDS_Face face = TopoDS::Face(faceExp.Current());
// //         TopLoc_Location loc;

// //         TopAbs_Orientation faceOrientation = face.Orientation();
// //         bool isReversed = (faceOrientation == TopAbs_REVERSED);

// //         Handle(Poly_Triangulation) triangulation = BRep_Tool::Triangulation(face, loc);

// //         if (!triangulation.IsNull()) {

// //             //Map of vertex indices in face to indices in model
// //             std::map<int, int> face_to_model_map;


// //             std::cout << "Location transformation for part: " << loc.Transformation().TranslationPart().X()
// //                        << ", " << loc.Transformation().TranslationPart().Y()
// //                        << ", " << loc.Transformation().TranslationPart().Z() << std::endl;

// //             // Convert OpenCascade vertices to CGAL points
// //             for (int i = 1; i <= triangulation->NbNodes(); ++i) {
// //                 gp_Pnt p = triangulation->Node(i).Transformed(loc.Transformation());
// //                 gp_Pnt q = triangulation->Node(i);

// //                 std::cout << "Transformed Point: " << p.X() << " " << p.Y() << " " << p.Z() << std::endl;

// //                 std::cout << "Untransformed Point: " << q.X() << " " << q.Y() << " " << q.Z() << std::endl;
                
// //                 //gp_Pnt p = triangulation->Node(i);
// //                 CGAL::Point_3<Kernel> cgalPoint(p.X(), p.Y(), p.Z());

// //                 if (vertexMap.find(p) == vertexMap.end()) {
// //                     vertexMap[p] = mesh.vertices.size();
// //                     mesh.vertices.push_back(cgalPoint);
// //                 }
                
// //                 face_to_model_map[i] = vertexMap[p];
// //             }

// //             // Extract indexed triangle faces
// //             for (int i = 1; i <= triangulation->NbTriangles(); ++i) {
// //                 Poly_Triangle tri = triangulation->Triangle(i);
// //                 int v1, v2, v3;

// //                 if (isReversed)
// //                     tri.Get(v1, v3, v2);

// //                 else
// //                     tri.Get(v1, v2, v3);
                
// //                 //mesh.faces.push_back({v1 - 1, v2 - 1, v3 - 1}); // Convert to 0-based indexing
// //                 mesh.faces.push_back({face_to_model_map[v1], face_to_model_map[v2], face_to_model_map[v3]}); // Convert to 0-based indexing

// //             }
// //         }
// //     }
// //     return mesh;
// // }










// std::string ModelLoader::GetShapeName(const TDF_Label& label) {
//     Handle(TDataStd_Name) nameAttr;
//     if (!label.FindAttribute(TDataStd_Name::GetID(), nameAttr)) {
//         return "Unnamed Shape";
//     }

//     // Convert ExtendedString to ASCII
//     TCollection_AsciiString asciiName(nameAttr->Get());

//     return asciiName.ToCString();
// }

// std::vector<std::shared_ptr<Part>> ModelLoader::loadSTEP2(const std::string& filename)
// {
//     std::vector<std::shared_ptr<Part>> parts;

//     points_.clear();

//     points_index_ = 0;


//     // Display connection
//     Handle(Aspect_DisplayConnection) displayConnection = new Aspect_DisplayConnection();

//     // Graphic driver (OpenGL)
//     Handle(Graphic3d_GraphicDriver) graphicDriver = new OpenGl_GraphicDriver(displayConnection);

//     // Viewer
//     Handle(V3d_Viewer) viewer = new V3d_Viewer(graphicDriver);
//     viewer->SetDefaultLights();
//     viewer->SetLightOn();

//     // View
//     Handle(V3d_View) view = viewer->CreateView();

//     // Create an interactive context
//     Handle(AIS_InteractiveContext) context = new AIS_InteractiveContext(viewer);


//     // Load STEP file using STEPCAFControl_Reader (handles metadata properly)
//     Handle(TDocStd_Document) doc = new TDocStd_Document("STEP");
//     STEPCAFControl_Reader stepReader;

//     if (stepReader.ReadFile(filename.c_str()) != IFSelect_RetDone) {
//         std::cerr << "Error: Failed to read STEP file." << std::endl;
//         return parts;
//     }

//     // Transfer to document structure
//     stepReader.Transfer(doc);
    
//     // Get the shape tool (manages multiple parts)
//     Handle(XCAFDoc_ShapeTool) shapeTool = XCAFDoc_DocumentTool::ShapeTool(doc->Main());
//     if (shapeTool.IsNull()) {
//         std::cerr << "Error: Unable to retrieve shape tool from document!" << std::endl;
//         return parts;
//     }


//     TDF_LabelSequence freeShapes;

//     shapeTool->GetFreeShapes(freeShapes);  // top-level shapes (assemblies or parts)


//     for (Standard_Integer i = 1; i <= freeShapes.Length(); ++i) {
//         TDF_Label assemblyLabel = freeShapes.Value(i);

//         std::string name = GetShapeName(assemblyLabel);

//         std::cout << "level_0_name: " << name << std::endl;

//         TDF_LabelSequence children_1;

//         shapeTool->GetComponents(assemblyLabel, children_1);

//         STEPControl_Writer writer;

//         for (Standard_Integer j = 1; j <= children_1.Length(); ++j)
//         {
//             TDF_Label child_1_label = children_1.Value(j);

//             std::string child_1_name = GetShapeName(child_1_label);

//             std::cout << "level_1_name: " << child_1_name << std::endl;

//             TopoDS_Shape child_1_shape = shapeTool->GetShape(child_1_label);

//             GProp_GProps props;
//             BRepGProp::VolumeProperties(child_1_shape, props);

//             gp_Pnt center = props.CentreOfMass();

//             std::cout << "CoM: " << center.X() << " " << center.Y() << " " << center.Z() << std::endl;


//             TopLoc_Location location = child_1_shape.Location();
//             gp_Trsf trsf = location.Transformation();

//             gp_XYZ translation = trsf.TranslationPart();

//             std::cout << "Loc: " << translation.X() << " " << translation.Y() << " " << translation.Z() << std::endl;


//             gp_Trsf trans_trsf;
//             trans_trsf.SetTranslation(gp_Vec(0, 0, (j - 1) * 20)); // Translate by (10, 0, 0)

//             TopoDS_Shape newShape = child_1_shape.Moved(trans_trsf);
//             //shapeTool->SetShape(label, newShape);

//             writer.Transfer(newShape, STEPControl_AsIs);

//             // Wrap shape in an AIS_Shape
//             Handle(AIS_Shape) aisShape = new AIS_Shape(newShape);
//             context->Display(aisShape, Standard_True);

//         }

//         //TCollection_AsciiString filename("OC_test_output.step");

//         writer.Write("OC_test_output.step");
//     }


//     // Create a window (X11 version here; use WNT_Window for Windows)
//     Handle(Xw_Window) window = new Xw_Window(displayConnection, "OpenCascade Viewer", 0, 0, 800, 600);
//     view->SetWindow(window);
//     view->SetBackgroundColor(Quantity_NOC_GRAY50);
//     view->MustBeResized();
//     view->TriedronDisplay(Aspect_TOTP_LEFT_LOWER, Quantity_NOC_WHITE, 0.1, V3d_ZBUFFER);
//     view->FitAll();

//     window->Map();

//     // Start the event loop — OpenCascade has no built-in loop, so use your GUI framework (Qt, wxWidgets) or system events
//     while (true) {
//         // simple event loop for demonstration purposes
//         view->Redraw();
//     }

//     return parts;




















//     TDF_Label rootLabel = doc->Main();




//     TDF_Label shapeLabelRoot = shapeTool->BaseLabel();


//     // std::cout << "==== Shape Tree ====\n";
//     // //TraverseAndPrintShapes(shapeTool, rootLabel, gp_Trsf());
//     // TraverseAndPrintShapes(shapeTool, shapeLabelRoot, gp_Trsf());

//     PrintLabelTree(shapeLabelRoot);



//     TDF_LabelSequence shapeLabels;

//     shapeTool->GetShapes(shapeLabels);

//     // // Display connection
//     // Handle(Aspect_DisplayConnection) displayConnection = new Aspect_DisplayConnection();

//     // // Graphic driver (OpenGL)
//     // Handle(Graphic3d_GraphicDriver) graphicDriver = new OpenGl_GraphicDriver(displayConnection);

//     // // Viewer
//     // Handle(V3d_Viewer) viewer = new V3d_Viewer(graphicDriver);
//     // viewer->SetDefaultLights();
//     // viewer->SetLightOn();

//     // // View
//     // Handle(V3d_View) view = viewer->CreateView();

//     // // Create an interactive context
//     // Handle(AIS_InteractiveContext) context = new AIS_InteractiveContext(viewer);

//     // // Dummy shape for demo (replace with your actual TopoDS_Shape)
//     // //TopoDS_Shape shape = BRepPrimAPI_MakeBox(100, 60, 40).Shape();

//     // TopoDS_Shape full_shape = shapeTool->GetShape(shapeLabelRoot);

//     // // Wrap shape in an AIS_Shape
//     // Handle(AIS_Shape) aisShape = new AIS_Shape(full_shape);
//     // //context->Display(aisShape, Standard_True);

//     // context->Display(aisShape, Standard_False); // Don't update immediately
//     // //context->SetDisplayMode(aisShape, AIS_Shaded, Standard_True); // Set to solid shaded mode
//     // context->SetDisplayMode(aisShape, AIS_WireFrame, Standard_True); // Set to solid shaded mode


//     // // Create a window (X11 version here; use WNT_Window for Windows)
//     // Handle(Xw_Window) window = new Xw_Window(displayConnection, "OpenCascade Viewer", 0, 0, 800, 600);
//     // view->SetWindow(window);
//     // view->SetBackgroundColor(Quantity_NOC_GRAY50);
//     // view->MustBeResized();
//     // view->TriedronDisplay(Aspect_TOTP_LEFT_LOWER, Quantity_NOC_WHITE, 0.1, V3d_ZBUFFER);
//     // view->FitAll();

//     // window->Map();

//     // // Start the event loop — OpenCascade has no built-in loop, so use your GUI framework (Qt, wxWidgets) or system events
//     // while (true) {
//     //     // simple event loop for demonstration purposes
//     //     view->Redraw();
//     // }













//     // for (Standard_Integer i = 1; i <= freeShapes.Length(); ++i) {
//     //     TDF_Label assemblyLabel = freeShapes.Value(i);

//     //     std::cout << "==== Shape Tree ====\n";
//     //     //TraverseAndPrintShapes(shapeTool, rootLabel, gp_Trsf());
//     //     TraverseAndPrintShapes(shapeTool, assemblyLabel, gp_Trsf(), 0);

//     //     std::cout << "Top-level shape:" << std::endl;

//     //     TraverseAssembly(shapeTool, assemblyLabel, gp_Trsf(), parts);  // recursion starts here
//     // }

//     std::cout << "NUM PARTS: " << parts.size() << std::endl;

//     // Iterate through all components


//     std::cout << "Number of components: " << shapeLabels.Length() << std::endl;

//     // for (Standard_Integer i = 1; i <= shapeLabels.Length(); ++i) {  //TODO
    
//     //     TDF_Label label = shapeLabels.Value(i);
//     //     TopoDS_Shape shape = shapeTool->GetShape(label);

//     //     // // Get name of the part
//     //     std::string shapeName = GetShapeName(label);
//     //     std::cout << shapeName << std::endl;

//     //     // Get the instance transformation **from the assembly**
//     //     TopLoc_Location instanceLocation = shape.Location();
//     //     gp_Trsf instanceTransform = instanceLocation.Transformation();

//     //     std::cout << "Instance transformation: "
//     //             << instanceTransform.TranslationPart().X() << ", "
//     //             << instanceTransform.TranslationPart().Y() << ", "
//     //             << instanceTransform.TranslationPart().Z() << std::endl;

//     // }

//     // //Move one of the shapes
//     // for (Standard_Integer i = 1; i <= shapeLabels.Length(); ++i) {  //TODO


//     //     TDF_Label label = shapeLabels.Value(i);
//     //     TopoDS_Shape shape = shapeTool->GetShape(label);

//     //     std::string shapeName = GetShapeName(label);

//     //     if (shapeName.find("external") != std::string::npos)
//     //     {
//     //         gp_Trsf trsf;
//     //         trsf.SetTranslation(gp_Vec(0, 0, 10)); // Translate by (10, 0, 0)

//     //         TopoDS_Shape newShape = shape.Moved(trsf);
//     //         shapeTool->SetShape(label, newShape);
//     //     }
//     // }

//     for (Standard_Integer i = 1; i <= shapeLabels.Length(); ++i) {  //TODO


//         TDF_Label label = shapeLabels.Value(i);
//         TopoDS_Shape shape = shapeTool->GetShape(label);


//         // // Display connection
//         // Handle(Aspect_DisplayConnection) displayConnection = new Aspect_DisplayConnection();

//         // // Graphic driver (OpenGL)
//         // Handle(Graphic3d_GraphicDriver) graphicDriver = new OpenGl_GraphicDriver(displayConnection);

//         // // Viewer
//         // Handle(V3d_Viewer) viewer = new V3d_Viewer(graphicDriver);
//         // viewer->SetDefaultLights();
//         // viewer->SetLightOn();

//         // // View
//         // Handle(V3d_View) view = viewer->CreateView();

//         // // Create an interactive context
//         // Handle(AIS_InteractiveContext) context = new AIS_InteractiveContext(viewer);

//         // // Dummy shape for demo (replace with your actual TopoDS_Shape)
//         // //TopoDS_Shape shape = BRepPrimAPI_MakeBox(100, 60, 40).Shape();

//         // // Wrap shape in an AIS_Shape
//         // Handle(AIS_Shape) aisShape = new AIS_Shape(shape);
//         // //context->Display(aisShape, Standard_True);

//         // context->Display(aisShape, Standard_False); // Don't update immediately
//         // context->SetDisplayMode(aisShape, AIS_Shaded, Standard_True); // Set to solid shaded mode


//         // // Create a window (X11 version here; use WNT_Window for Windows)
//         // Handle(Xw_Window) window = new Xw_Window(displayConnection, "OpenCascade Viewer", 0, 0, 800, 600);
//         // view->SetWindow(window);
//         // view->SetBackgroundColor(Quantity_NOC_GRAY50);
//         // view->MustBeResized();
//         // view->TriedronDisplay(Aspect_TOTP_LEFT_LOWER, Quantity_NOC_WHITE, 0.1, V3d_ZBUFFER);
//         // view->FitAll();

//         // window->Map();

//         // // Start the event loop — OpenCascade has no built-in loop, so use your GUI framework (Qt, wxWidgets) or system events
//         // while (true) {
//         //     // simple event loop for demonstration purposes
//         //     view->Redraw();
//         // }




//         // // Get name of the part
//         std::string shapeName = GetShapeName(label);
//         std::cout << std::endl << std::endl << "Component " << i << ": " << shapeName << std::endl;


//         // Apply the location (transformation) to the shape
//         TopLoc_Location location = shape.Location();
//         TopoDS_Shape locatedShape = shape;


//         gp_Trsf trsf = location.Transformation();

//         // Print translation
//         gp_XYZ translation = trsf.TranslationPart();
//         std::cout << "Translation: "
//                 << translation.X() << ", "
//                 << translation.Y() << ", "
//                 << translation.Z() << std::endl;

//         // Print rotation matrix
//         gp_Mat rotation = trsf.VectorialPart();
//         std::cout << "Rotation matrix:\n";
//         std::cout << rotation.Value(1,1) << " " << rotation.Value(1,2) << " " << rotation.Value(1,3) << "\n";
//         std::cout << rotation.Value(2,1) << " " << rotation.Value(2,2) << " " << rotation.Value(2,3) << "\n";
//         std::cout << rotation.Value(3,1) << " " << rotation.Value(3,2) << " " << rotation.Value(3,3) << std::endl;


        

//         if (!location.IsIdentity()) {
//             locatedShape = shape.Moved(location);
//         }

//         else
//             std::cout << "Location is identity" << std::endl;

//         // Deep copy of the shape
//         //BRepBuilderAPI_Copy copier(locatedShape);
//         //TopoDS_Shape copiedShape = copier.Shape();

//         //parts.push_back(std::shared_ptr<Part>(new Part()));
//         //parts.back()->setShape(std::make_shared(copiedShape))

 
//         // Part::PART_TYPE type;

//         // if (shapeName.find("internal") != std::string::npos)
//         //     type = Part::INTERNAL;

//         // else if (shapeName.find("external") != std::string::npos)
//         //     type = Part::EXTERNAL;

//         // else if (shapeName.find("screw") != std::string::npos)
//         //     type = Part::SCREW;

//         // else
//         //     continue;

//         // parts.push_back(std::shared_ptr<Part>(new Part(std::make_shared<TopoDS_Shape>(copiedShape), type, i, shapeName)));
     


//         // // Get the instance transformation **from the assembly**
//         // TopLoc_Location instanceLocation = shape.Location();
//         // gp_Trsf instanceTransform = instanceLocation.Transformation();

//         // std::cout << "Instance transformation: "
//         //         << instanceTransform.TranslationPart().X() << ", "
//         //         << instanceTransform.TranslationPart().Y() << ", "
//         //         << instanceTransform.TranslationPart().Z() << std::endl;




//         // BRepBuilderAPI_NurbsConvert convert(shape);
//         // convert.Perform(shape);
//         // if (convert.IsDone()) {
//         //     shape = convert.Shape();  // The shape is now in NURBS format
//         // }
//         // else {
//         //     std::cerr << "NURBS conversion failed!" << std::endl;
//         // }
        
//         // ShapeFix_Shape fixer(shape);
//         // fixer.Perform();
//         // // if (fixer.IsDone()) {
//         // //     shape = fixer.Shape();  // The shape is now fixed
//         // // }
//         // // else {
//         // //     std::cerr << "ShapeFix_Shape failed!" << std::endl;
//         // // }

//         // // BRepCheck_Analyzer analyzer(shape);
//         // // if (analyzer.IsValid()) {
//         // //     std::cout << "Shape is valid." << std::endl;
//         // // } else {
//         // //     std::cout << "Shape has issues:" << std::endl;
//         // //     for (Standard_Integer i = 1; i <= analyzer.NbErrors(); ++i) {
//         // //         std::cout << analyzer.Error(i) << std::endl;
//         // //     }
//         // // }




//         // gp_Trsf trsf;
//         // trsf.SetTranslation(gp_Vec(30, 0, 0)); // Translate by (10, 0, 0)
//         // TopoDS_Shape moved_shape = BRepBuilderAPI_Transform(shape, trsf).Shape();




//         // TopoDS_Shape cradle = BRepPrimAPI_MakeBox(100, 100, 15); // Base box
 
//         // // Perform the subtraction: A - B
//         // BRepAlgoAPI_Cut cut(cradle, moved_shape);
//         // cut.Build();

//         // if (!cut.IsDone()) {
//         //     std::cerr << "Cut operation failed!" << std::endl;
//         // }

//         // TopoDS_Shape result = cut.Shape();

//         // BRepMesh_IncrementalMesh(result, 0.1);  // Mesh with a 0.1 tolerance

//         // // Export the result to STL
//         // StlAPI_Writer cut_writer;
//         // cut_writer.Write(result, (shapeName + "_cut_result.stl").c_str());



//         BRepMesh_IncrementalMesh(shape, 0.1);  // Mesh with a 0.1 tolerance

//         // Set up the STL writer
//         StlAPI_Writer writer;
//         writer.Write(shape, (shapeName + std::string("_OC.stl")).c_str());


    


//         // std::ifstream in((shapeName + std::string("_OC.stl")).c_str(), std::ios::binary);


//         // std::shared_ptr<SurfaceMesh> surface_mesh = std::shared_ptr<SurfaceMesh>(new SurfaceMesh());

//         // CGAL::IO::read_STL(in, *surface_mesh);


//         // in.close();


//         // // Extract mesh with the correct instance transform
//         // TriangleMesh mesh = ExtractMeshFromShape(shape, instanceTransform, i);


//         // std::shared_ptr<SurfaceMesh> surface_mesh = std::shared_ptr<SurfaceMesh>(new SurfaceMesh());

//         // std::vector<SurfaceMesh::Vertex_index> vertex_map;

//         // for (unsigned int i = 0; i < mesh.vertices.size(); ++i) {

//         //     CGAL::Point_3<Kernel> v = mesh.vertices[i];
//         //     vertex_map.push_back(surface_mesh->add_vertex(Point(v.x(), v.y(), v.z())));
//         // }

//         // for (unsigned int i = 0; i < mesh.faces.size(); ++i) {
//         //     std::array<int, 3> face = mesh.faces[i];
//         //     surface_mesh->add_face(vertex_map[face[0]],
//         //                 vertex_map[face[1]],
//         //                 vertex_map[face[2]]);
//         // }



//         // double xmin = std::numeric_limits<double>::max();
//         // double ymin = std::numeric_limits<double>::max();
//         // double zmin = std::numeric_limits<double>::max();
//         // double xmax = std::numeric_limits<double>::lowest();
//         // double ymax = std::numeric_limits<double>::lowest();
//         // double zmax = std::numeric_limits<double>::lowest();



//         // for (auto v : surface_mesh->vertices()){
//         // //for (auto v = surface_mesh->vertices_begin(); v != surface_mesh->vertices_end(); ++v) {
            
//         //     auto p = surface_mesh->point(v);

//         //     xmin = std::min(xmin, CGAL::to_double(p.x()));
//         //     ymin = std::min(ymin, CGAL::to_double(p.y()));
//         //     zmin = std::min(zmin, CGAL::to_double(p.z()));
//         //     xmax = std::max(xmax, CGAL::to_double(p.x()));
//         //     ymax = std::max(ymax, CGAL::to_double(p.y()));
//         //     zmax = std::max(zmax, CGAL::to_double(p.z()));
//         // }

//         // std::cout << "BBox: " << xmin << " - " << xmax << ", " << ymin << " - " << ymax << ", " << zmin << " - " << zmax << std::endl; 

//         // std::shared_ptr<NamedSurfaceMesh> named_surface_mesh  = std::shared_ptr<NamedSurfaceMesh>(new NamedSurfaceMesh());

//         // named_surface_mesh->mesh = surface_mesh;


//         // //Set shape name to lower case
//         // std::transform(shapeName.begin(), shapeName.end(), shapeName.begin(), [](unsigned char c){ return std::tolower(c); });

//         // named_surface_mesh->name = shapeName;
        
//         // named_meshes.push_back(named_surface_mesh);


//         // //Mesh fixes

//         // CGAL::Polygon_mesh_processing::remove_isolated_vertices(*surface_mesh);

//         // CGAL::Polygon_mesh_processing::stitch_borders(*surface_mesh);

//         // // std::vector<halfedge_descriptor> border_cycles;
//         // // CGAL::Polygon_mesh_processing::extract_boundary_cycles(*surface_mesh, border_cycles);

//         // // for (halfedge_descriptor h : border_cycles)
//         // //     CGAL::Polygon_mesh_processing::triangulate_and_refine_hole(mesh, h);


//         //saveMesh(surface_mesh, shapeName + ".stl");
//     }

//     return parts;
// }

// std::vector<std::shared_ptr<NamedPolyhedron>> ModelLoader::loadSTEP(const std::string& filename)
// {
//     points_.clear();

//     points_index_ = 0;


//     std::vector<std::shared_ptr<NamedPolyhedron>> named_meshes;

//     // Load STEP file using STEPCAFControl_Reader (handles metadata properly)
//     Handle(TDocStd_Document) doc = new TDocStd_Document("STEP");
//     STEPCAFControl_Reader stepReader;
    
//     if (stepReader.ReadFile(filename.c_str()) != IFSelect_RetDone) {
//         std::cerr << "Error: Failed to read STEP file." << std::endl;
//         return named_meshes;
//     }

//     // Transfer to document structure
//     stepReader.Transfer(doc);

//     // Get the shape tool (manages multiple parts)
//     Handle(XCAFDoc_ShapeTool) shapeTool = XCAFDoc_DocumentTool::ShapeTool(doc->Main());
//     if (shapeTool.IsNull()) {
//         std::cerr << "Error: Unable to retrieve shape tool from document!" << std::endl;
//         return named_meshes;
//     }

//     // Iterate through all components
//     TDF_LabelSequence shapeLabels;
//     shapeTool->GetShapes(shapeLabels);

//     std::cout << "Number of components: " << shapeLabels.Length() << std::endl;

//     for (Standard_Integer i = 1; i <= shapeLabels.Length(); ++i) {  //TODO
    
//         TDF_Label label = shapeLabels.Value(i);
//         TopoDS_Shape shape = shapeTool->GetShape(label);

//         // // Get name of the part
//         std::string shapeName = GetShapeName(label);
//         std::cout << shapeName << std::endl;

//         // Get the instance transformation **from the assembly**
//         TopLoc_Location instanceLocation = shape.Location();
//         gp_Trsf instanceTransform = instanceLocation.Transformation();

//         std::cout << "Instance transformation: "
//                 << instanceTransform.TranslationPart().X() << ", "
//                 << instanceTransform.TranslationPart().Y() << ", "
//                 << instanceTransform.TranslationPart().Z() << std::endl;

//     }

//     for (Standard_Integer i = 1; i <= shapeLabels.Length(); ++i) {  //TODO


//         TDF_Label label = shapeLabels.Value(i);
//         TopoDS_Shape shape = shapeTool->GetShape(label);

//         // // Get name of the part
//         std::string shapeName = GetShapeName(label);
//         std::cout << std::endl << std::endl << "Component " << i << ": " << shapeName << std::endl;

//         // Get the instance transformation **from the assembly**
//         TopLoc_Location instanceLocation = shape.Location();
//         gp_Trsf instanceTransform = instanceLocation.Transformation();

//         std::cout << "Instance transformation: "
//                 << instanceTransform.TranslationPart().X() << ", "
//                 << instanceTransform.TranslationPart().Y() << ", "
//                 << instanceTransform.TranslationPart().Z() << std::endl;

//         BRepMesh_IncrementalMesh(shape, 0.1);  // Mesh with a 0.1 tolerance

//         // Extract mesh with the correct instance transform
//         TriangleMesh mesh = ExtractMeshFromShape(shape, instanceTransform, i);


//         // if (shapeName.find("internal") == std::string::npos && 
//         //     shapeName.find("external") == std::string::npos &&
//         //     shapeName.find("screw") == std::string::npos)
//         //     continue;

//         // TDF_Label label = shapeLabels.Value(i);
//         // TopoDS_Shape shape = shapeTool->GetShape(label);

//         // // Get name of the part
//         // std::string shapeName = GetShapeName(label);
//         // std::cout << "Component " << i << ": " << shapeName << std::endl;


//         ///////

//         //BRepMesh_IncrementalMesh(shape, 0.1);  // Mesh with a 0.1 tolerance


//         // // **Get the transformation of this part in the assembly**
//         // TopLoc_Location partLocation = shape.Location();
//         // gp_Trsf partTransform = partLocation.Transformation();

//         // std::cout << "Part transformation: " 
//         //         << partTransform.TranslationPart().X() << ", "
//         //         << partTransform.TranslationPart().Y() << ", "
//         //         << partTransform.TranslationPart().Z() << std::endl;

//         // // Generate a mesh for the part, applying part-level transformation
//         // TriangleMesh mesh = ExtractMeshFromShape(shape, partTransform);

//         ////////////

//         // // Generate a mesh for the part
//         // BRepMesh_IncrementalMesh(shape, 0.1);  // Mesh with a 0.1 tolerance

//         // //Set up CGAL mesh
//         // TriangleMesh mesh = ExtractMeshFromShape(shape);

//         //Analyse mesh
//         std::cout << std::endl << "Mesh:" << std::endl;
//         std::cout << "Num vertices: " << mesh.vertices.size() << std::endl;
//         std::cout << "Num faces: " << mesh.faces.size() << std::endl;


//         Polyhedron P;
//         BuildPolyhedron<Polyhedron::HalfedgeDS> builder(mesh);
//         P.delegate(builder);

//         std::shared_ptr<Polyhedron> polyhedron = std::make_shared<Polyhedron>(P);


//         std::cout << "Loaded mesh " << shapeName << " at " << meshCenter(polyhedron).x() << " " << meshCenter(polyhedron).y() << " " << meshCenter(polyhedron).z() << std::endl; 

//         std::cout << "X span: " << meshBoundingBox(polyhedron).xmin() << " " << meshBoundingBox(polyhedron).xmax() << std::endl;
//         std::cout << "Y span: " << meshBoundingBox(polyhedron).ymin() << " " << meshBoundingBox(polyhedron).ymax() << std::endl;
//         std::cout << "Z span: " << meshBoundingBox(polyhedron).zmin() << " " << meshBoundingBox(polyhedron).zmax() << std::endl;


//         std::shared_ptr<NamedPolyhedron> named_polyhedron  = std::shared_ptr<NamedPolyhedron>(new NamedPolyhedron());

//         named_polyhedron->polyhedron = polyhedron;


//         //Set shape name to lower case
//         std::transform(shapeName.begin(), shapeName.end(), shapeName.begin(), [](unsigned char c){ return std::tolower(c); });

//         named_polyhedron->name = shapeName;
        
//         named_meshes.push_back(named_polyhedron);

//         if (!CGAL::is_closed(*polyhedron)) {
//             std::cout << "Mesh has open boundaries!" << std::endl;

//             continue;

//         } else {
//             std::cout << "Mesh is closed!" << std::endl;
//         }

//         std::vector<Polyhedron::Halfedge_handle> border_edges;
//         CGAL::Polygon_mesh_processing::border_halfedges(*polyhedron, std::back_inserter(border_edges));
//         std::cout << "Number of open edges: " << border_edges.size() << std::endl;

//         // Stitch close edges together
//         //CGAL::Polygon_mesh_processing::remove_self_intersections(*polyhedron);
//         CGAL::Polygon_mesh_processing::stitch_borders(*polyhedron);
//         //CGAL::Polygon_mesh_processing::orient_polygon_soup(*polyhedron); 

//         if (!CGAL::is_closed(*polyhedron)) {
//             std::cout << "Mesh has open boundaries!" << std::endl;
//         } else {
//             std::cout << "Mesh is closed!" << std::endl;
//         }

//         std::vector<Polyhedron::Halfedge_handle> border_edges2;
//         CGAL::Polygon_mesh_processing::border_halfedges(*polyhedron, std::back_inserter(border_edges2));
//         std::cout << "Number of open edges: " << border_edges2.size() << std::endl;

//         std::cout << "Successfully converted STEP to CGAL Polyhedron with "
//               << polyhedron->size_of_vertices() << " vertices and "
//               << polyhedron->size_of_facets() << " faces" << std::endl;
//     }

//     return named_meshes;
// }

// void ProcessNode(const aiNode* node, const aiScene* scene)
// {
//     std::cout << "Node name: " << node->mName.C_Str() << std::endl;

//     for (unsigned int i; i < node->mNumMeshes; i++)
//     {
//         unsigned int meshIndex = node->mMeshes[i];
//         aiMesh* mesh = scene->mMeshes[meshIndex];
//         std::cout << "Mesh index: " << meshIndex << " (Vertices: " << mesh->mNumVertices << ")" << std::endl;
//     }

//     for (unsigned int i = 0; i < node->mNumChildren; i++) {
//         ProcessNode(node->mChildren[i], scene);
//     }
// }

// std::shared_ptr<Assembly> ModelLoader::loadModel(const std::string& filename) {

//     // Assimp::Importer importer;

//     // const aiScene* scene = importer.ReadFile("MotorMountTest v8.amf", aiProcess_Triangulate | aiProcess_JoinIdenticalVertices);

//     // if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode)
//     // {
//     //     std::cerr << "Assimp Error: " << importer.GetErrorString() << std::endl;
//     // }

//     // for (unsigned int i = 0; i < scene->mNumMeshes; i++) {
//     //     aiMesh* mesh = scene->mMeshes[i];

//     //     std::cout << "Mesh name: " << mesh->mName.C_Str() << std::endl;
//     // }

//     // ProcessNode(scene->mRootNode, scene);
    


//     // // Display connection
//     // Handle(Aspect_DisplayConnection) displayConnection = new Aspect_DisplayConnection();

//     // // Graphic driver (OpenGL)
//     // Handle(Graphic3d_GraphicDriver) graphicDriver = new OpenGl_GraphicDriver(displayConnection);

//     // // Viewer
//     // Handle(V3d_Viewer) viewer = new V3d_Viewer(graphicDriver);
//     // viewer->SetDefaultLights();
//     // viewer->SetLightOn();

//     // // View
//     // Handle(V3d_View) view = viewer->CreateView();

//     // // Create an interactive context
//     // Handle(AIS_InteractiveContext) context = new AIS_InteractiveContext(viewer);

//     // // Dummy shape for demo (replace with your actual TopoDS_Shape)
//     // TopoDS_Shape shape = BRepPrimAPI_MakeBox(100, 60, 40).Shape();

//     // // Wrap shape in an AIS_Shape
//     // Handle(AIS_Shape) aisShape = new AIS_Shape(shape);
//     // context->Display(aisShape, Standard_True);

//     // // Create a window (X11 version here; use WNT_Window for Windows)
//     // Handle(Xw_Window) window = new Xw_Window(displayConnection, "OpenCascade Viewer", 0, 0, 800, 600);
//     // view->SetWindow(window);
//     // view->SetBackgroundColor(Quantity_NOC_GRAY50);
//     // view->MustBeResized();
//     // view->TriedronDisplay(Aspect_TOTP_LEFT_LOWER, Quantity_NOC_WHITE, 0.1, V3d_ZBUFFER);
//     // view->FitAll();

//     // window->Map();

//     // // Start the event loop — OpenCascade has no built-in loop, so use your GUI framework (Qt, wxWidgets) or system events
//     // while (true) {
//     //     // simple event loop for demonstration purposes
//     //     view->Redraw();
//     // }



//     //Create new assembly
//     std::shared_ptr<Assembly> assembly = std::shared_ptr<Assembly>(new Assembly());

//     //std::vector<std::shared_ptr<NamedPolyhedron>> named_polyhedrons = loadSTEP(filename);
    
//     std::vector<std::shared_ptr<Part>> parts = loadSTEP2(filename);

//     for (std::shared_ptr<Part> part : parts)
//         assembly->addPart(part);

//     assembly->saveAsSTL("target_assembly.stl");

//     // std::vector<SurfaceMesh> meshes;
    
//     // CGAL::IO::read_3MF("MotorMountTest v8.2mf", meshes);

//     // int i = 0;

//     // for (auto mesh : meshes)
//     // {
//     //     std::stringstream ss;

//     //     ss << "3mf_mesh_" << i << ".stl";

//     //     saveMesh(mesh, ss.str());

//     //     i ++;
//     // }

//     //TODO
//     // int id = 0;

//     // for (std::shared_ptr<NamedPolyhedron> named_polyhedron : named_polyhedrons)
//     // {
//     //     Part::PART_TYPE type;

//     //     if (named_polyhedron->name.find("internal") != std::string::npos)
//     //         type = Part::INTERNAL;

//     //     else if (named_polyhedron->name.find("external") != std::string::npos)
//     //         type = Part::EXTERNAL;

//     //     else if (named_polyhedron->name.find("screw") != std::string::npos)
//     //         type = Part::SCREW;

//     //     else
//     //         continue;

//     //     std::shared_ptr<Part> part = std::shared_ptr<Part>(new Part(named_polyhedron->polyhedron, type, id, named_polyhedron->name));

//     //     assembly->addPart(part);

//     //     id ++;
//     // }      

//     return assembly;
// }


// std::shared_ptr<MeshObject> ModelLoader::loadSubstrate(const std::string& filename)
// {
//     std::cout << "Loading substrate" << std::endl;

//     std::shared_ptr<MeshObject> substrate = std::shared_ptr<MeshObject>(new MeshObject());

//     std::vector<std::shared_ptr<NamedPolyhedron>> named_polyhedrons = loadSTEP(filename);

//     for (auto named_polyhedron : named_polyhedrons)
//     {
//         std::cout << "Substrate polyhedron: " << named_polyhedron->name << std::endl;
//     }

//     if (named_polyhedrons.size() != 1)
//     {
//         std::cout << "Wrong number of polyhedrons in substrate" << std::endl;
//         return substrate;
//     }

//     substrate->setMesh(named_polyhedrons[0]->polyhedron);

//     return substrate;
// }