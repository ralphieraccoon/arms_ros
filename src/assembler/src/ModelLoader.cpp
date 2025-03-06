#include "assembler/ModelLoader.hpp"
#include "assembler/Assembly.hpp"
#include "assembler/Part.hpp"

#include <iostream>

//TODO Just load the single aiScene as before and have an assembly class that points to a list of meshes in that assembly and just make sure to put them
//back in the correct position after wiggling them about to do the test (will mostly happen in other library anyway) or visualization

ModelLoader::ModelLoader() {}

std::shared_ptr<Assembly> ModelLoader::loadModel(const std::string& filepath) 
{
    //Create new assembly
    std::shared_ptr<Assembly> assembly = std::shared_ptr<Assembly>(new Assembly());

    const aiScene* scene = importer_.ReadFile(filepath, aiProcess_Triangulate);

    if (!scene) {
        std::cout << "Scene not loaded" << std::endl;
            return assembly;
    }

    std::cout << "Scene loaded" << std::endl;    


    aiNode* rootNode = scene->mRootNode;


    std::cout << "Num meshes in scene: " << scene->mNumMeshes << std::endl;

    for (unsigned int i = 0; i < scene->mNumMeshes; i ++)
    {
        std::cout << "Mesh. Num bones: " << scene->mMeshes[i]->mNumBones << " Num faces: " << scene->mMeshes[i]->mNumFaces
                << " Has normals: " << scene->mMeshes[i]->HasNormals() << " Has positions: " << scene->mMeshes[i]->HasPositions() << std::endl;

        float x, y, z = 0;

        for (unsigned int j = 0; j < scene->mMeshes[i]->mNumVertices; j++)
        {
            x += scene->mMeshes[i]->mVertices[j].x;
            y += scene->mMeshes[i]->mVertices[j].y;
            z += scene->mMeshes[i]->mVertices[j].z;
        }

        x = x / scene->mMeshes[i]->mNumVertices;
        y = y / scene->mMeshes[i]->mNumVertices;
        z = z / scene->mMeshes[i]->mNumVertices;

        std::cout << "Average mesh position: " << x << " " << y << " " << z << std::endl;
    }

    std::cout << "Name of root node: " << rootNode->mName.C_Str() << std::endl;

    std::cout << "Num meshes in root node: " << rootNode->mNumMeshes << std::endl;

    std::cout << "Num children in root node: " << rootNode->mNumChildren << std::endl;

    aiNode* topNode = rootNode->mChildren[0];

    std::cout << "Name of top node: " << topNode->mName.C_Str() << std::endl;

    std::cout << "Num meshes in top node: " << topNode->mNumMeshes << std::endl;

    std::cout << "Num children in top node: " << topNode->mNumChildren << std::endl;

    //Get mesh positions

    std::cout << "Top level node down:" << std::endl;

    for (unsigned int i = 0; i < topNode->mNumChildren; i ++)
    {   
        std::cout << std::endl;

        aiNode* nameNode = topNode->mChildren[i];


        std::cout << "Node. Name: " << nameNode->mName.C_Str() << " meshes: " << nameNode->mNumMeshes << " children: " << nameNode->mNumChildren << " Transform: " 
                    << nameNode->mTransformation.a4 << " " 
                    << nameNode->mTransformation.b4 << " " 
                    << nameNode->mTransformation.c4 << std::endl;

        for (unsigned int j = 0; j < nameNode->mNumChildren; j ++)
        {
            aiNode* nameChildNode = nameNode->mChildren[j];

            std::cout << "Node. Name: " << nameChildNode->mName.C_Str() << " meshes: " << nameChildNode->mNumMeshes << " children: " << nameChildNode->mNumChildren << " Transform: " 
                    << nameChildNode->mTransformation.a4 << " " 
                    << nameChildNode->mTransformation.b4 << " " 
                    << nameChildNode->mTransformation.c4 << std::endl;

            for (unsigned int k = 0; k < nameChildNode->mNumChildren; k ++)
            {
                aiNode* nameChildChildNode = nameChildNode->mChildren[k];
    
                std::cout << "Node. Name: " << nameChildChildNode->mName.C_Str() << " meshes: " << nameChildChildNode->mNumMeshes << " children: " << nameChildChildNode->mNumChildren << " Transform: " 
                        << nameChildChildNode->mTransformation.a4 << " " 
                        << nameChildChildNode->mTransformation.b4 << " " 
                        << nameChildChildNode->mTransformation.c4 << std::endl;
            }
        }

    }


    //Iterate through each mesh, create a new part including a CGAL mesh and add it to the assembly
    for (size_t i = 0; i < scene->mNumMeshes; i ++)
    {
        std::shared_ptr<Polyhedron> polyhedron = std::shared_ptr<Polyhedron>(new Polyhedron());

        Part::PART_TYPE type = Part::EXTERNAL;

        aiMesh* mesh = scene->mMeshes[i];

        BuildPolyhedron<Polyhedron::HalfedgeDS> builder(mesh);
        polyhedron->delegate(builder);

        std::cout << "Polyhedron has " << polyhedron->size_of_vertices() << " vertices and "
              << polyhedron->size_of_facets() << " faces." << std::endl;

        std::shared_ptr<Part> part = std::shared_ptr<Part>(new Part(polyhedron, type, i));

        assembly->addPart(part);

        std::cout << "Part centroid: " << part->getCentroid().x() << " " << part->getCentroid().y() << " " << part->getCentroid().z() << std::endl;
    }

    return assembly;
}


// std::shared_ptr<Polyhedron> ModelLoader::loadSubstrate(const std::string& filepath)
// {
//     Assimp::Importer importer;

//     const aiScene* scene = importer.ReadFile(filepath, aiProcess_Triangulate);

//     std::shared_ptr<Polyhedron> polyhedron = std::shared_ptr<Polyhedron>(new Polyhedron());


//     if (!scene) {
//         std::cout << "Substrate scene not loaded" << std::endl;
//         return polyhedron;
//     }

//     std::cout << "Substrate scene loaded" << std::endl;    

//     if (scene->mNumMeshes != 1)
//     {
//         std::cout << "Incorrect number of meshes in substrate scene" << std::endl;
//         return polyhedron;
//     }
    
//     aiMesh* mesh = scene->mMeshes[0];

//     BuildPolyhedron<Polyhedron::HalfedgeDS> builder(mesh);
//     polyhedron->delegate(builder);
    
//     return polyhedron;
// }