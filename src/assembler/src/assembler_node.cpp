#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
// #include <assimp/Importer.hpp>
// #include <assimp/scene.h>
// #include <assimp/postprocess.h>

#include "assembler/ModelLoader.hpp"
#include "assembler/Assembler.hpp"
#include "assembler/Assembly.hpp"
#include "assembler/Part.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <iostream>

class AssemblerNode : public rclcpp::Node {
public:
    AssemblerNode() : Node("model_loader"), loader_(std::make_shared<ModelLoader>()), assembler_(std::make_shared<Assembler>()) 
    {
        RCLCPP_INFO(this->get_logger(), "Model loader node starting");

        std::string inputPath = ament_index_cpp::get_package_share_directory("assembler") + "/../../../../";

        // Create a publisher for RViz visualization
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/model_loader/mesh", 10);

        loadMesh(inputPath);

        //publishMesh(inputPath);

        //assembler_->generateAssembly();



        // Timer to publish at 1 Hz
        //timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&ModelLoaderNode::publishMesh, this));
    }

private:

     void loadMesh(std::string inputPath)
     {
        //loader_->loadSTEP(inputPath + "FileFormatTest v2.step");
        //loader_->loadSTEP(inputPath + "MotorMountTest v1.step");


        std::shared_ptr<Assembly> initial_assembly = loader_->loadModel(inputPath + "MotorMountTest v2.step");


        //std::shared_ptr<Assembly> initial_assembly = loader_->loadModel(inputPath + "TestPlate.step");

        //std::shared_ptr<Assembly> initial_assembly = loader_->loadModel(inputPath + "MotorMountTest v1.stl");

        assembler_->setSubstrate(loader_->loadSubstrate(inputPath + "Parts_bay_socket_12mm v2.step"));


        //assembler_->setSubstrate(loader_->loadSubstrate(inputPath + "Parts_bay_socket_6mm v0.fbx"));

        assembler_->setInitialAssembly(initial_assembly);

        assembler_->generateAssemblySequence();
     }

    void publishMesh(std::string inputPath) {

        std::shared_ptr<Assembly> assembly = loader_->loadModel(inputPath + "MotorMountTest v2.step");

        //const aiScene* scene = loader_->loadModel("FileFormatTest v2.fbx");

        //assembler_->generateAssembly(scene);

        // Create a visualization marker
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";  // Change to match your RViz2 frame
        marker.header.stamp = this->now();
        marker.ns = "model";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Set marker properties
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        // Process Assimp mesh data
        //aiMesh* mesh = scene->mMeshes[0];

        std::cout << "Check1" << std::endl;

        std::shared_ptr<Part> part = assembly->getParts()[1];

        std::cout << "Check2" << std::endl;

        std::shared_ptr<Polyhedron> mesh = part->getMesh();

        std::cout << "Check3" << std::endl;
        
        for (auto face = mesh->facets_begin(); face != mesh->facets_end(); ++face) {
            std::vector<geometry_msgs::msg::Point> face_points;
        
            std::cout << "Check4" << std::endl;

            // Ensure the face has a valid halfedge
            if (face->halfedge() == nullptr) {
                std::cerr << "Error: Face has a null halfedge, skipping!" << std::endl;
                continue;
            }

            auto halfedge = face->halfedge();
            do {

                if (halfedge == nullptr) {
                    std::cerr << "Error: Encountered a null halfedge, skipping face!" << std::endl;
                    break; // Skip this face entirely
                }

                CGAL::Polyhedron_3<CGAL::Epeck>::Vertex_handle vh = halfedge->vertex();
        
                std::cout << "Check5" << std::endl;

                // Ensure vertex exists and has a valid point
                if (vh == nullptr) {  
                    std::cerr << "Warning: Encountered an invalid vertex!" << std::endl;
                    continue;  // Skip this halfedge
                }
        

                std::cout << "Check6" << std::endl;

                geometry_msgs::msg::Point p;
                const auto& point = vh->point();  // CGAL Point_3
                
                std::cout << "Check7" << std::endl;

                p.x = CGAL::to_double(point.x());
                p.y = CGAL::to_double(point.y());
                p.z = CGAL::to_double(point.z());
                face_points.push_back(p);
        
                std::cout << "Check8" << std::endl;

                halfedge = halfedge->next();
            } while (halfedge != face->halfedge());
        
            std::cout << "Check9" << std::endl;

            // Ensure it's a triangle before adding to the marker
            if (face_points.size() == 3) {
                marker.points.push_back(face_points[0]);
                marker.points.push_back(face_points[1]);
                marker.points.push_back(face_points[2]);
            } else {
                std::cerr << "Warning: Encountered a non-triangle face with " 
                          << face_points.size() << " vertices, skipping!" << std::endl;
            }
        }


        // for (unsigned int i = 0; i < mesh->mNumFaces; i++) {

        //     aiFace face = mesh->mFaces[i];
        //     for (int j = 0; j < 3; j++) {  // Triangular faces
        //         geometry_msgs::msg::Point p;
        //         aiVector3D vertex = mesh->mVertices[face.mIndices[j]];
        //         p.x = vertex.x;
        //         p.y = vertex.y;
        //         p.z = vertex.z;
        //         marker.points.push_back(p);
        //     }
        // }

        // Publish the marker
        marker_pub_->publish(marker);
        RCLCPP_INFO(this->get_logger(), "Published model to /model_loader/mesh");
    }

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<Assembler> assembler_;
    std::shared_ptr<ModelLoader> loader_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AssemblerNode>());
    rclcpp::shutdown();
    return 0;
}

// #include <rclcpp/rclcpp.hpp>
// #include <visualization_msgs/msg/marker.hpp>
// #include <sensor_msgs/msg/point_cloud2.hpp>
// #include <CGAL/Surface_mesh.h>
// #include <CGAL/Simple_cartesian.h>
// #include <CGAL/Polygon_mesh_processing/corefinement.h>
// //#include <CGAL/IO/STL_reader.h>
// #include <assimp/Importer.hpp>
// #include <assimp/scene.h>
// #include <assimp/postprocess.h>
// #include <geometry_msgs/msg/point.hpp>
// #include <geometry_msgs/msg/quaternion.hpp>

// typedef CGAL::Simple_cartesian<double> Kernel;
// typedef CGAL::Surface_mesh<Kernel::Point_3> Mesh;

// class ModelLoaderNode : public rclcpp::Node
// {
// public:
//     ModelLoaderNode() : Node("model_loader_node")
//     {
//         marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);

//         // Load a 3D model from file
//         loadModel("FileFormatTest v2.fbx");
//     }

//     void createMarker(const Mesh &mesh)
//     {
//         auto marker = visualization_msgs::msg::Marker();
//         marker.header.frame_id = "base_link";
//         marker.header.stamp = this->get_clock()->now();
//         marker.ns = "model";
//         marker.id = 0;
//         marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
//         marker.action = visualization_msgs::msg::Marker::ADD;
//         marker.scale.x = 1.0;
//         marker.scale.y = 1.0;
//         marker.scale.z = 1.0;
//         marker.color.a = 1.0;
//         marker.color.r = 1.0;
//         marker.color.g = 1.0;
//         marker.color.b = 1.0;

//         // Loop over all the faces in the mesh
//         for (const auto &face : mesh.faces())
//         {
//             geometry_msgs::msg::Point p1, p2, p3;
//             auto h1 = mesh.halfedge(face);
//             auto h2 = mesh.next(h1);
//             auto h3 = mesh.next(h2);

//             p1.x = mesh.point(mesh.target(h1)).x();
//             p1.y = mesh.point(mesh.target(h1)).y();
//             p1.z = mesh.point(mesh.target(h1)).z();

//             p2.x = mesh.point(mesh.target(h2)).x();
//             p2.y = mesh.point(mesh.target(h2)).y();
//             p2.z = mesh.point(mesh.target(h2)).z();

//             p3.x = mesh.point(mesh.target(h3)).x();
//             p3.y = mesh.point(mesh.target(h3)).y();
//             p3.z = mesh.point(mesh.target(h3)).z();

//             marker.points.push_back(p1);
//             marker.points.push_back(p2);
//             marker.points.push_back(p3);
//         }

//         marker_pub_->publish(marker);
//     }

//     void loadModel(const std::string &file_path)
//     {
//         Assimp::Importer importer;
//         const aiScene *scene = importer.ReadFile(file_path, aiProcess_Triangulate | aiProcess_FlipUVs);

//         if (!scene || !scene->HasMeshes())
//         {
//             RCLCPP_ERROR(this->get_logger(), "Assimp could not load the model.");
//             return;
//         }

//         Mesh mesh;

//         // Parse the meshes and convert to CGAL format
//         for (unsigned int i = 0; i < scene->mNumMeshes; ++i)
//         {
//             aiMesh *ai_mesh = scene->mMeshes[i];
//             for (unsigned int j = 0; j < ai_mesh->mNumFaces; ++j)
//             {
//                 aiFace &face = ai_mesh->mFaces[j];
//                 for (unsigned int k = 0; k < face.mNumIndices; ++k)
//                 {
//                     int idx = face.mIndices[k];
//                     auto vertex = ai_mesh->mVertices[idx];
//                     mesh.add_vertex(Kernel::Point_3(vertex.x, vertex.y, vertex.z));
//                 }
//             }
//         }

//         createMarker(mesh);
//     }

// private:
//     rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
// };

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<ModelLoaderNode>());
//     rclcpp::shutdown();
//     return 0;
// }


