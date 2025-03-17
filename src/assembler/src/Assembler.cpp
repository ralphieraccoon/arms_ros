#include "assembler/Assembler.hpp"

#include "assembler/Assembly.hpp"


#include "assembler/Part.hpp"

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include "yaml-cpp/yaml.h"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <iostream>
#include <fstream>
#include <queue>
#include <set>
#include <map>

Assembler::Assembler()
{
    //TODO
    output_path_ = ament_index_cpp::get_package_share_directory("assembler") + "/../../../../";
    input_path_ = ament_index_cpp::get_package_share_directory("assembler") + "/../../../../";

    std::cout << "Ouput path: " << output_path_ << std::endl;
    std::cout << "Input path: " << input_path_ << std::endl;

    std::shared_ptr<Assembly> assembly = std::shared_ptr<Assembly>(new Assembly());
}

void Assembler::generateAssemblySequence() 
{
    if (initial_assembly_ == nullptr)
        return;

    generateNegatives();

    std::cout << "Generating assembly" << std::endl;


    YAML::Node root;

    YAML::Node commands = YAML::Node(YAML::NodeType::Sequence);

    std::vector<std::shared_ptr<Part>> initial_parts = initial_assembly_->getParts();

    for (std::shared_ptr<Part> part : initial_parts)
    {        
        if (!part->getType() == Part::EXTERNAL)
            continue;

        YAML::Node detect_part_command;
        detect_part_command["command-type"] = "LOCATE_EXTERNAL_PART";
        detect_part_command["command-properties"]["part-description"] = ""; //TODO
        detect_part_command["command-properties"]["part-name"] = ""; //TODO
        detect_part_command["command-properties"]["part-id"] = part->getId();
        detect_part_command["command-properties"]["part-height"] = 100; //TODO

        commands.push_back(detect_part_command);
    }

    root["commands"] = commands;

    std::ofstream fout(Assembler::output_path_ + "assembly_plan.yaml");

    fout << root;

    fout.close();


    std::vector<std::shared_ptr<AssemblyNode>> path = breadthFirstZAssembly();

    // // Process Assimp mesh data
    // aiMesh* mesh = scene->mMeshes[0];

        

    // for (unsigned int i = 0; i < mesh->mNumFaces; i++) {

    //     aiFace face = mesh->mFaces[i];

    //     std::cout << "Face" << std::endl;

    //     for (int j = 0; j < 3; j++) {  // Triangular faces

    //     }
    // }
}

std::vector<std::shared_ptr<AssemblyNode>> Assembler::breadthFirstZAssembly()
{
    std::vector<std::shared_ptr<AssemblyNode>> path;

    std::shared_ptr<AssemblyNode> target_node;

    std::shared_ptr<AssemblyNode> first_part_node;

    std::shared_ptr<AssemblyNode> base_node = std::shared_ptr<AssemblyNode>(new AssemblyNode());

    base_node->assembly_ = initial_assembly_;

    base_node->id_ = nodeIdGenerator(initial_assembly_->getPartIds());

    std::queue<std::shared_ptr<AssemblyNode>> queue;

    std::set<std::shared_ptr<AssemblyNode>> visited_nodes;

    std::map<std::shared_ptr<AssemblyNode>, std::shared_ptr<AssemblyNode>> parents;

    queue.push(base_node);

    visited_nodes.emplace(base_node);

    std::cout << "starting breadth search" << std::endl;

    while (queue.size() > 0)
    {
        std::shared_ptr<AssemblyNode> current_node = queue.front();

        std::cout << "Current node: " << current_node->id_ << std::endl;
 
        queue.pop();

        //Check if current node is target node (e.g., has no meshes)
        if (current_node->assembly_->getParts().size() == 0)               
            target_node = current_node;

        std::vector<std::shared_ptr<AssemblyNode>> neighbours = findNodeNeighbours(current_node);

        for (std::shared_ptr<AssemblyNode> neighbour : neighbours)
        {
            std::cout << "neighbour: " << neighbour->id_ << std::endl; 

            //If neighbour has already been visited, move on
            if (visited_nodes.find(neighbour) != visited_nodes.end())
                continue;

            std::cout << "not visited" << std::endl;

            visited_nodes.emplace(neighbour);

            queue.push(neighbour);

            parents[neighbour] = current_node;
        }
    }

    if (!parents.count(target_node))
    {
        std::cout << "No path found" << std::endl;

        return path;
    }

    std::shared_ptr<AssemblyNode> path_node = target_node;

    path.push_back(target_node);

    while (parents.count(path_node))
    {
        path_node = parents[path_node];

        path.push_back(path_node);
    }

    //TODO - find internal first node

    std::cout << std::endl << "Found path" << std::endl << std::endl; 

    for (std::shared_ptr<AssemblyNode> node : path)
    {
        std::cout << "node id: " << node->id_ << " parts:" << std::endl;
        
        for (std::shared_ptr<Part> part : node->assembly_->getParts())
        {
            Point centroid = part->getCentroid();

            std::cout << centroid.x() << " " << centroid.y() << " " << centroid.z() << std::endl;
        }

        std::cout << std::endl;
    }

    return path;
}

std::vector<std::shared_ptr<AssemblyNode>> Assembler::findNodeNeighbours(std::shared_ptr<AssemblyNode> node)
{
    std::vector<std::shared_ptr<AssemblyNode>> neighbours;


    std::vector<std::shared_ptr<Part>> parts = node->assembly_->getParts();

    //Iterate through each part
    //Try to move the part vertically over 10cm
    //Every 0.1cm check collisions with every other part
    //If no collisions take place, create a new assemblynode with this part removed and add it to neighbours list
    float step_size = 0.1f;


    for (std::shared_ptr<Part> part : parts)
    {
        bool collides = false;

        int num_steps = 0;

        for (int i = 0; i != 100; i ++)
        {   
            num_steps ++;

            part->translate(Vector(0, 0, step_size));

            for (std::shared_ptr<Part> otherPart : parts)
            {
                //Don't try to collide with self
                if (otherPart->getId() == part->getId())
                    continue;

                Point cd1 = part->getCentroid();

                Point cd2 = otherPart->getCentroid();

                std::cout << "Checking collision: " << part->getId() << " and "  << otherPart->getId() << std::endl;

                std::cout << "Z Positions: " << cd1.z() << " | " << cd2.z() << std::endl;

                if (part->collide(otherPart))
                {
                    std::cout << "Collision" << std::endl;

                    collides = true;
                    break;
                }
            }

            if (collides)
                break;
        }

        //Put the part back where it was
        part->translate(Vector(0, 0, -num_steps * step_size));

        if (collides)
            continue;

        std::cout << std::endl;

        //Create new assembly node
        std::shared_ptr<Assembly> neighbour_assembly = std::shared_ptr<Assembly>(new Assembly());

        std::shared_ptr<AssemblyNode> neighbour_node = std::shared_ptr<AssemblyNode>(new AssemblyNode());

        for (std::shared_ptr<Part> part_to_add : parts)
        {
            //Don't copy over part to be removed
            if (part_to_add->getId() == part->getId())
                continue;

            neighbour_assembly->addPart(part_to_add->clone());
        }

        neighbour_node->assembly_ = neighbour_assembly;

        neighbour_node->id_ = nodeIdGenerator(neighbour_assembly->getPartIds());

        neighbours.push_back(neighbour_node);
        
    } 



    // float max_z = 0;
    // std::shared_ptr<Part> max_z_part;

    // for (std::shared_ptr<Part> part : parts)
    // {
    //     if (part->getCentroid().z() >= max_z)
    //     {
    //         max_z = part->getCentroid().z();

    //         max_z_part = part;
    //     }
    // }

    // //No assembly can be a neighbour
    // if (max_z_part == nullptr)
    //     return neighbours;

    // std::shared_ptr<Assembly> neighbour_assembly = std::shared_ptr<Assembly>(new Assembly());

    // std::shared_ptr<AssemblyNode> neighbour_node = std::shared_ptr<AssemblyNode>(new AssemblyNode());

    // for (std::shared_ptr<Part> part : parts)
    // {
    //     //Don't copy over part to be removed
    //     if (part == max_z_part)
    //         continue;

    //     neighbour_assembly->addPart(part->clone());
    // }

    // neighbour_node->assembly_ = neighbour_assembly;

    // neighbour_node->id_ = nodeIdGenerator(neighbour_assembly->getPartIds());

    // neighbours.push_back(neighbour_node);


    return neighbours;
}

size_t Assembler::nodeIdGenerator(std::vector<size_t> object_ids)
{
    for (auto const& [id, obj_ids] : node_id_map_)
    {
        if (object_ids.size() != obj_ids.size())
            continue;
        
        bool allFound = true;

        for (size_t obj_id_1 : object_ids)
        {
            bool found = false;

            for (size_t obj_id_2 : obj_ids)
            {
                if (obj_id_1 == obj_id_2)
                {
                    found = true;
                    break;
                }
            }

            if (!found)
            {
                allFound = false;

                break;
            }
        }

        if (allFound)
            return id;
    }

    //Not found, new ID
    node_id_map_[next_node_ID_++] = object_ids;

    return next_node_ID_ - 1;
}

void Assembler::generateNegatives()
{
    negative_substrate_->centerMesh();

    int i = 0;

    for (std::shared_ptr<Part> part : initial_assembly_->getParts())
    {
        //Only create negatives or external parts
        if (!part->getType() == Part::EXTERNAL)
            continue;

        std::stringstream ss;

        ss << "negative_" << i << "_" << part->getName() << ".stl";

        part->centerMesh();

        part->createNegative(negative_substrate_, ss.str());

        i ++;
    }
}