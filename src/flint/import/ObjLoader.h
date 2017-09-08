
#pragma once

#include <tiny_obj_loader.h>
#include "geometry/Mesh.h"
#include "geometry/Triangle.h"

namespace import {

// template <typename GeometryType>
// void LoadObj(const std::string &inputfile, geometry::Mesh<3, GeometryType>* mesh);
template <typename GeometryType>
struct ObjLoader {
    void Load(const std::string &inputfile, geometry::Mesh<3, GeometryType>* mesh);
};

template <typename T>
struct TriangleObjLoader {
    void Load(const std::string &inputfile, geometry::Mesh<3, geometry::Triangle<3, T>>* mesh) {
        tinyobj::attrib_t attrib;
        std::vector<tinyobj::shape_t> shapes;
        std::vector<tinyobj::material_t> materials;
    
        std::string err;
        bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &err, inputfile.c_str());
    
        if (!err.empty()) { // `err` may contain warning message.
            std::cerr << err << std::endl;
        }
    
        if (!ret) {
            return;
        }
    
        // Loop over shapes
        for (size_t s = 0; s < shapes.size(); s++) {
            // Loop over faces(polygon)
            size_t index_offset = 0;
            for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
                size_t fv = shapes[s].mesh.num_face_vertices[f];
    
                // Loop over and triangulate vertices in the face.
                for (size_t v = 1; v < fv - 1; v++) {
                    std::array<Eigen::Matrix<float, 3, 1>, 3> points;
    
                    std::array<size_t, 3> vs = {0, v, v + 1};
                    for (unsigned int i = 0; i < 3; ++i) {
                        // access to vertex
                        tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + vs[i]];
                        tinyobj::real_t vx = attrib.vertices[3*idx.vertex_index+0];
                        tinyobj::real_t vy = attrib.vertices[3*idx.vertex_index+1];
                        tinyobj::real_t vz = attrib.vertices[3*idx.vertex_index+2];
                        // tinyobj::real_t nx = attrib.normals[3*idx.normal_index+0];
                        // tinyobj::real_t ny = attrib.normals[3*idx.normal_index+1];
                        // tinyobj::real_t nz = attrib.normals[3*idx.normal_index+2];
                        // tinyobj::real_t tx = attrib.texcoords[2*idx.texcoord_index+0];
                        // tinyobj::real_t ty = attrib.texcoords[2*idx.texcoord_index+1];
    
                        points[i] = Eigen::Vector3f(vx, vy, vz);
                    }
    
                    mesh->AddGeometry(new geometry::Triangle<3, T>(points));
                }
    
                index_offset += fv;
            
                // per-face material
                shapes[s].mesh.material_ids[f];
            }
        }
    }
};


// template <typename GeometryType>
// std::enable_if<std::is_same<GeometryType, 


}