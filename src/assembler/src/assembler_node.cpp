#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "assembler/ModelLoader.hpp"
#include "assembler/Assembler.hpp"
#include "assembler/Assembly.hpp"
#include "assembler/Part.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <iostream>

class AssemblerNode : public rclcpp::Node {
public:
    AssemblerNode(const std::string &filename) : Node("model_loader"), loader_(std::make_shared<ModelLoader>()), assembler_(std::make_shared<Assembler>()) 
    {
        RCLCPP_INFO(this->get_logger(), "Model loader node starting");

        std::string inputPath = ament_index_cpp::get_package_share_directory("assembler") + "/../../../../";

        // Create a publisher for RViz visualization
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/model_loader/mesh", 10);

        loadMesh(inputPath + filename);

        //publishMesh(inputPath);

        //assembler_->generateAssembly();



        // Timer to publish at 1 Hz
        //timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&ModelLoaderNode::publishMesh, this));
    }

private:

    void loadMesh(std::string inputPath)
    {
        std::shared_ptr<Assembly> initial_assembly = loader_->loadModel(inputPath);

        assembler_->setSubstrate(loader_->loadSubstrate(inputPath + "Parts_bay_socket_6mm v2.step"));

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

    std::string arg_value = "default_value";

    if (argc > 1) {
        arg_value = argv[1];  // Read first argument (excluding program name)
    }

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AssemblerNode>(arg_value));
    rclcpp::shutdown();
    return 0;
}