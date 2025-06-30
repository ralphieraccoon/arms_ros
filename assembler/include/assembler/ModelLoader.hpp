#ifndef MODEL_LOADER_HPP
#define MODEL_LOADER_HPP

#include <memory>

#include <boost/property_map/property_map.hpp>

#include "assembler/Part.hpp"

class MeshObject;

class Assembly;

class ModelLoader {

    public:

        static std::shared_ptr<Assembly> loadModel(const std::string& filename);

    private:

        static std::vector<std::shared_ptr<Part>> loadSTEP(const std::string& filename);

        static std::string GetShapeName(const TDF_Label& label);

        static void RecurrentAddPart(const TDF_Label& label, const Handle(XCAFDoc_ShapeTool)& shapeTool, std::vector<std::shared_ptr<Part>>& parts, int level);

        static int next_id_;
};

#endif  // MODEL_LOADER_HPP