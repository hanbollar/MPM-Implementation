
#include <iostream>
#include <Eigen/Dense>
#include "flint/import/ObjLoader.h"

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Expected one argument" << std::endl;
        return 1;
    }

    using Triangle = geometry::Triangle<3, float>;

    auto* mesh = new geometry::Mesh<3, Triangle>();
    import::TriangleObjLoader<float> loader;
    loader.Load(argv[1], mesh);
    std::cout << "Mesh has " << mesh->geometries().size() << " triangles" << std::endl;
    delete mesh;
    return 0;
}
