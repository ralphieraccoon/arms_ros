#ifndef MODEL_LOADER_HPP
#define MODEL_LOADER_HPP

#include <memory>

#include <boost/property_map/property_map.hpp>

#include "assembler/Part.hpp"

class MeshObject;

class Assembly;



class ModelLoader {

    public:

        explicit ModelLoader() {}
        std::shared_ptr<Assembly> loadModel(const std::string& filename);

    private:

        std::vector<std::shared_ptr<Part>> loadSTEP(const std::string& filename);

        std::string GetShapeName(const TDF_Label& label);

        void RecurrentAddPart(const TDF_Label& label, const Handle(XCAFDoc_ShapeTool)& shapeTool, std::vector<std::shared_ptr<Part>>& parts, int level);

        int next_id_ = 0;
};
    
    


// struct TriangleMesh {
//     std::vector<CGAL::Point_3<Kernel>> vertices;
//     std::vector<std::array<int, 3>> faces; // Indices into vertices list
// };

// struct PointComparator {
//     bool operator()(const gp_Pnt& p1, const gp_Pnt& p2) const {
//         if (p1.X() != p2.X()) return p1.X() < p2.X();
//         if (p1.Y() != p2.Y()) return p1.Y() < p2.Y();
//         return p1.Z() < p2.Z();
//     }
// };

// //Wrapper class for naming meshes that are loaded with the loadSTEP function
// struct NamedPolyhedron {
//     std::shared_ptr<Polyhedron> polyhedron;
//     std::string name;
// };

// //Wrapper class for naming meshes that are loaded with the loadSTEP function
// struct NamedSurfaceMesh {
//     std::shared_ptr<SurfaceMesh> mesh;
//     std::string name;
// };

// class ModelLoader {
// public:
//     explicit ModelLoader() {}
//     std::shared_ptr<Assembly> loadModel(const std::string& filename);

//     std::shared_ptr<MeshObject> loadSubstrate(const std::string& filename);

//     std::vector<std::shared_ptr<NamedPolyhedron>> loadSTEP(const std::string& filename);

//     std::vector<std::shared_ptr<Part>> loadSTEP2(const std::string& filename);

//     std::string GetShapeName(const TDF_Label& label);

//     TriangleMesh ExtractMeshFromShape(const TopoDS_Shape& shape, const gp_Trsf& partTransform, int shape_index);

// private:
//     std::vector<double> points_;

//     int points_index_ = 0;

//     //TriangleMesh ExtractMeshFromShape(const TopoDS_Shape& shape);
// };

// template <class HDS>
// class BuildPolyhedron : public CGAL::Modifier_base<HDS> {
//     const TriangleMesh& mesh;

// public:
//     BuildPolyhedron(const TriangleMesh& mesh) : mesh(mesh) {}

//     void operator()(HDS& hds) {
//         CGAL::Polyhedron_incremental_builder_3<HDS> builder(hds, true);
//         builder.begin_surface(mesh.vertices.size(), mesh.faces.size());

//         // Add vertices
//         for (const auto& v : mesh.vertices) {
//             builder.add_vertex(v);
//         }

//         // Add faces
//         for (const auto& f : mesh.faces) {
//             //builder.add_facet(f[0], f[1], f[2]);

//             builder.begin_facet();
//             builder.add_vertex_to_facet(f[0]);
//             builder.add_vertex_to_facet(f[1]);
//             builder.add_vertex_to_facet(f[2]);
//             builder.end_facet();
//         }

//         builder.end_surface();
//     }
// };

#endif  // MODEL_LOADER_HPP