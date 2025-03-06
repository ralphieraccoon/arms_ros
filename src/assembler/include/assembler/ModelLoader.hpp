#ifndef MODEL_LOADER_HPP
#define MODEL_LOADER_HPP

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <memory>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>

class Substrate;

class Assembly;

class ModelLoader {
public:
    explicit ModelLoader();
    std::shared_ptr<Assembly> loadModel(const std::string& filepath);

    std::shared_ptr<Substrate> loadSubstrate(const std::string& filepath);

private:
    Assimp::Importer importer_;
};

template <class HDS>
class BuildPolyhedron : public CGAL::Modifier_base<HDS> {
public:
    const aiMesh* ai_mesh;

    BuildPolyhedron(const aiMesh* mesh) : ai_mesh(mesh) {}

    void operator()(HDS& hds) {
        typedef typename HDS::Vertex::Point Point;  // Ensure correct type resolution

        CGAL::Polyhedron_incremental_builder_3<HDS> builder(hds, true);
        builder.begin_surface(ai_mesh->mNumVertices, ai_mesh->mNumFaces);

        // Add vertices
        for (unsigned int i = 0; i < ai_mesh->mNumVertices; ++i) {
            aiVector3D v = ai_mesh->mVertices[i];
            builder.add_vertex(Point(v.x, v.y, v.z));  // Use the correct Point type
        }

        // Add faces
        for (unsigned int i = 0; i < ai_mesh->mNumFaces; ++i) {
            const aiFace& face = ai_mesh->mFaces[i];
            if (face.mNumIndices == 3) {  // Ensure it's a triangle
                builder.begin_facet();
                builder.add_vertex_to_facet(face.mIndices[0]);
                builder.add_vertex_to_facet(face.mIndices[1]);
                builder.add_vertex_to_facet(face.mIndices[2]);
                builder.end_facet();
            }
        }

        builder.end_surface();
    }
};

#endif  // MODEL_LOADER_HPP