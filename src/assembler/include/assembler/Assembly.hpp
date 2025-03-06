#ifndef ASSEMBLY_HPP
#define ASSEMBLY_HPP

//#include <CGAL/Simple_cartesian.h>
//#include <CGAL/Surface_mesh.h>

#include <memory>
#include <vector>
#include <set>

class Part;

class Assembly {

public:
    explicit Assembly();

    void addPart(std::shared_ptr<Part> part) { parts_.push_back(part); }
 
    std::vector<std::shared_ptr<Part>> getParts() { return parts_; }

    std::vector<size_t> getPartIds();

private:
    std::vector<std::shared_ptr<Part>> parts_;

};

struct AssemblyNode {

    std::shared_ptr<Assembly> assembly_;

    size_t id_;

    std::set<size_t> parent_ids_;

    bool operator <(const AssemblyNode& rhs) const              //TODO this is janky
    {
        return id_ < rhs.id_;
    }

    bool operator ==(const AssemblyNode& rhs) const
    {
        return id_ == rhs.id_;
    }
};

#endif  // ASSEMBLY_HPP