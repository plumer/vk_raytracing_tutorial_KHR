//
//  meshloader.cpp
//  pbrt-learn


#include <cassert>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <numeric>
#ifdef _WIN32
#include <string>  // std::getline is only available in this file
#endif

#include "../common/types.h"
#include "logging.h"
#include "meshloader.h"
#include <glm/gtx/string_cast.hpp>

bool SanityCheck(const vkpbr::PlyMesh& m);

vkpbr::PlyMesh LoadTriangleMesh(const std::string& file_name)
{
    size_t         dot_pos  = file_name.find_last_of('.');
    std::string    ext_name = file_name.substr(dot_pos + 1);
    vkpbr::PlyMesh mesh_data;
    if (ext_name == "obj") {
        mesh_data = LoadObj(file_name, /*enforce_triangle=*/true);
    } else if (ext_name == "ply") {
        mesh_data = LoadPly(file_name, /*enforce_triangle=*/true);
    } else {
        LOG(ERROR) << "unexpected file extension name: " << ext_name;
        return {};
    }

    if (mesh_data.indices.empty() || SanityCheck(mesh_data) == false) {
        LOG(FATAL) << "mesh file corrupted";
        return {};
    }

    if (mesh_data.normals.size() != mesh_data.positions.size()) {
        // Fills in the normal vector data.
        mesh_data.normals = compute_normals(mesh_data.positions, mesh_data.indices);
        LOG(INFO) << "normal filled up";
    }

    return mesh_data;
}

vkpbr::PlyMesh LoadTriangleMeshBVH(const std::string& file_name, const glm::mat4& obj_to_world,
                                   bool reverse_orientation)
{
    std::filesystem::path file_path = file_name;
    vkpbr::PlyMesh        mesh_data;
    if (file_path.extension() == ".obj") {
        mesh_data = LoadObj(file_name, /*enforce_triangle=*/true);
    } else if (file_path.extension() == ".ply") {
        mesh_data = LoadPly(file_name, /*enforce_triangle=*/true);
    } else {
        LOG(ERROR) << "Unhandled file extension name: " << file_path.extension();
        return {};
    }

    if (mesh_data.indices.empty() || SanityCheck(mesh_data) == false) {
        LOG(FATAL) << "mesh file corrupted";
        return {};
    }

    if (mesh_data.normals.size() != mesh_data.positions.size()) {
        // Fills in the normal vector data.
        mesh_data.normals = compute_normals(mesh_data.positions, mesh_data.indices);
        LOG(INFO) << "normal filled up";
    }

    if (mesh_data.texture_uvs.empty()) {
        LOG(ERROR) << "Mesh " << file_name << " doesn't have texture UV data";
    }

    return mesh_data;
}

std::vector<std::string> split(const std::string& s, char delimiter);

vkpbr::PlyMesh LoadObj(const std::string& filename, bool enforce_triangle)
{
    std::vector<glm::vec3> vertices;
    std::vector<int>       indices;

    std::ifstream objFile;
    objFile.open(filename, std::ios::in);
    assert(objFile.fail() == false);
    while (objFile.eof() == false) {
        std::string buffer;
        std::getline(objFile, buffer);
        auto words = split(buffer, ' ');
        if (words.empty())
            continue;  // empty line
        if (words.front()[0] == '#')
            continue;  // comment line
        if (words[0] == "v") {
            assert(words.size() == 4);
            float x = std::atof(words[1].c_str());
            float y = std::atof(words[2].c_str());
            float z = std::atof(words[3].c_str());
            vertices.push_back(glm::vec3{x, y, z});
        } else if (words[0] == "f") {
            std::vector<int> faceIndices;
            faceIndices.reserve(words.size() - 1);
            for (size_t i = 1; i < words.size(); ++i) {
                int index = std::atoi(words[i].c_str());
                faceIndices.push_back(index);
            }
            if (enforce_triangle && faceIndices.size() > 3) {
                decltype(faceIndices) triangleIndices;
                triangleIndices.reserve((faceIndices.size() - 2) * 3);
                // if there are 6 vertices, then the first triangle is [0, 1, 2], the last is [0, 4,
                // 5].
                for (size_t i = 1; i < faceIndices.size() - 1; ++i) {
                    triangleIndices.push_back(faceIndices[0]);
                    triangleIndices.push_back(faceIndices[i]);
                    triangleIndices.push_back(faceIndices[i + 1]);
                }
                std::swap(triangleIndices, faceIndices);
            }
            indices.insert(indices.end(), faceIndices.begin(), faceIndices.end());
        } else {
            std::cerr << "unhandled line starting with \'" << words[0] << "\'" << std::endl;
        }
    }

    int nPoints = vertices.size();
    for (int& index : indices) {
        // suppose nPoints is 8
        if (index < 0)
            index += nPoints;  // [-nPoints, -1]
        else
            index--;  // now index is [1, nPoints], decrement it.
    }

    vkpbr::PlyMesh res;
    res.positions = std::move(vertices);
    res.indices   = std::move(indices);
    return res;
}

int get_type_size(const std::string& s)
{
    if (s == "uchar")
        return 1;
    if (s == "short")
        return 2;
    if (s == "int")
        return 4;
    if (s == "uint")
        return 4;
    if (s == "uint8")
        return 1;
    else
        std::cerr << "Unhandled type string: " << s << std::endl;
    return 8;
}

enum class PLYFormat { eNone, eAscii, eBinaryLE, eBinaryBE };

vkpbr::PlyMesh LoadPly(const std::string& filename, bool enforce_triangle)
{
    std::ifstream reader;
    reader.open(filename, std::ios::in | std::ios::binary);
    if (!reader) {
        return {};
    }

    std::string              buffer;
    std::vector<std::string> words;

    // Reads the header.
    std::getline(reader, buffer);
    if (buffer != "ply") {
        std::cerr << "file is not ply; header = \'" << buffer << "\'" << std::endl;
        return {};
    }

    // Reads the next line.
    // ``` format ascii/binary_little_endian/binary_big_endian 1.0 ```
    std::getline(reader, buffer);
    words = split(buffer, ' ');
    if (words[0] != "format") {
        std::cerr << "Cannot read format at the second line."
                  << "The first word read is " << words[0] << std::endl;
        return {};
    }
    enum PLYFormat format = PLYFormat::eNone;

    if (words[1] == "ascii") {
        format = PLYFormat::eAscii;
    } else {
        std::vector<std::string> tokens = split(words[1], '_');
        if (tokens.size() != 3 || tokens[0] != "binary" || tokens[2] != "endian") {
            std::cerr << "ill formed format string: " << words[1] << std::endl;
            return {};
        }
        format = tokens[1] == "little" ? PLYFormat::eBinaryLE :
                                         ("big" ? PLYFormat::eBinaryBE : PLYFormat::eNone);

        assert(format != PLYFormat::eNone);
    }
    assert(words[2] == "1.0");

    int nVertices = 0;
    int nFaces    = 0;
    // assume all attributes are float
    std::vector<std::string> vertex_attrib_names;
    int                      nPointsOnFace           = 0;
    int                      list_length_number_size = 0;  // 1, 2, or 4.
    int                      list_element_size       = 0;  // 1, 2, or 4

    // lines that are Not Comments
    std::vector<std::string> nclines;
    do {
        if (reader.eof())
            return {};
        std::getline(reader, buffer);
        if (buffer.substr(0, 7) == "comment")
            continue;
        nclines.push_back(buffer);
    } while (buffer != "end_header");

    for (auto it = nclines.cbegin(); it != nclines.cend(); it++) {
        std::vector<std::string> words = split(*it, ' ');
        if (words.empty())
            continue;
        else if (*it == "end_header") {
            assert((it + 1) == nclines.end());
        } else if (words.size() < 3) {
            std::cerr << "can't handle the line: " << *it << std::endl;
        } else if (words.front() == "element" && words[1] == "vertex") {
            nVertices = std::stol(words[2]);
            // process the next few lines until a new element
            do {
                it++;
                words = split(*it, ' ');
                if (words[0] != "property")
                    break;
                CHECK(words[1] == "float");
                vertex_attrib_names.push_back(words[2]);
            } while (words[0] == "property");
            it--;
        } else if (words.front() == "element" && words[1] == "face") {
            nFaces = std::stol(words[2]);
            // process the next line
            it++;
            words = split(*it, ' ');
            CHECK(words[0] == "property") << "words[0] = " << words[0];
            CHECK(words[1] == "list") << "words[1] = " << words[1];
            list_length_number_size = get_type_size(words[2]);
            list_element_size       = get_type_size(words[3]);
        } else if (words.front() == "element") {
            std::cerr << "unhandled element" << words.front() << std::endl;
        } else {
            std::cerr << "unhandled line:" << *it << std::endl;
        }
    }

    // Reads binary data.
    // ----------------------------------------------------------------------------------------
    std::vector<float> vertex_buffer;
    vertex_buffer.resize(nVertices * vertex_attrib_names.size());
    reader.read(reinterpret_cast<char*>(vertex_buffer.data()),
                vertex_buffer.size() * sizeof(float));

    std::vector<int> indices_list;
    for (int face_i = 0; face_i < nFaces; ++face_i) {
        if (reader.eof()) {
            std::cerr << "reading ended prematurely" << std::endl;
            break;
        }
        int list_length = 0;
        // Reads the length of the list.
        reader.read(reinterpret_cast<char*>(&list_length), list_length_number_size);
        if (list_length <= 0) {
            std::cerr << "length of the list < 0: " << list_length << std::endl;
            break;
        }
        assert(list_element_size <= 4);
        // read `list_length` integers from the file.
        // Every integer takes `list_element_size` bytes, but probably not more than 4.
        std::vector<int> face_indices;
        for (int i = 0; i < list_length; ++i) {
            int index = 0;
            reader.read(reinterpret_cast<char*>(&index), list_element_size);
            face_indices.push_back(index);
        }
        if (enforce_triangle) {
            for (int i = 1; i < list_length - 1; ++i) {
                indices_list.push_back(face_indices[0]);
                indices_list.push_back(face_indices[i]);
                indices_list.push_back(face_indices[i + 1]);
            }
        } else {
            indices_list.insert(indices_list.end(), face_indices.begin(), face_indices.end());
        }
    }

    if (reader.eof()) {
        std::cout << "Read exactly to the end of file" << std::endl;
    }

    reader.close();
    // std::cout << "reading success" << std::endl;

    // >>>>>>>>>> Organize the data from vertex buffer to mesh data >>>>>>>>>>>

    // counted in number of components, not bytes
    int  stride    = vertex_attrib_names.size();
    auto van_begin = vertex_attrib_names.cbegin(), van_end = vertex_attrib_names.cend();
    int  offset_px = -1, offset_py = -1, offset_pz = -1, offset_nx = -1, offset_ny = -1,
        offset_nz = -1, offset_u = -1, offset_v = -1;
    for (int i = 0; i < stride; ++i) {
        if (vertex_attrib_names[i] == "x")
            offset_px = i;
        if (vertex_attrib_names[i] == "y")
            offset_py = i;
        if (vertex_attrib_names[i] == "z")
            offset_pz = i;
        if (vertex_attrib_names[i] == "nx")
            offset_nx = i;
        if (vertex_attrib_names[i] == "ny")
            offset_ny = i;
        if (vertex_attrib_names[i] == "nz")
            offset_nz = i;
        if (vertex_attrib_names[i] == "u")
            offset_u = i;
        if (vertex_attrib_names[i] == "v")
            offset_v = i;
    }

    bool has_normal = offset_nx >= 0 && offset_ny >= 0 && offset_nz >= 0;
    bool has_texuv  = offset_u >= 0 && offset_v >= 0;
    auto res        = vkpbr::PlyMesh();
    for (int base = 0; base < vertex_buffer.size(); base += stride) {
        glm::vec3 pos;
        glm::vec3 normal;
        glm::vec2 uv;
        if (offset_px >= 0)
            pos.x = vertex_buffer[base + offset_px];
        if (offset_py >= 0)
            pos.y = vertex_buffer[base + offset_py];
        if (offset_pz >= 0)
            pos.z = vertex_buffer[base + offset_pz];

        res.positions.push_back(pos);

        if (has_normal) {
            // if (offset_nx >= 0)
            normal.x = vertex_buffer[base + offset_nx];
            // if (offset_ny >= 0)
            normal.y = vertex_buffer[base + offset_ny];
            // if (offset_nz >= 0)
            normal.z = vertex_buffer[base + offset_nz];
            res.normals.push_back(normal);
        }

        if (has_texuv) {
            uv.x = vertex_buffer[base + offset_u];
            uv.y = vertex_buffer[base + offset_v];
            res.texture_uvs.push_back(uv);
        }
    }

    res.indices = std::move(indices_list);

    return res;
}

std::vector<glm::vec3> compute_normals(const std::vector<glm::vec3>& positions,
                                       const std::vector<int>&       tri_indices)
{
    std::vector<glm::vec3> res;
    const int              num_triangles = tri_indices.size() / 3;
    CHECK_EQ(num_triangles * 3, tri_indices.size());
    std::vector<std::vector<glm::vec3>> builder;
    builder.resize(positions.size());
    for (int tri_i = 0; tri_i < num_triangles; ++tri_i) {
        int i0 = tri_indices[tri_i * 3 + 0], i1 = tri_indices[tri_i * 3 + 1],
            i2       = tri_indices[tri_i * 3 + 2];
        glm::vec3 t0 = positions[i0], t1 = positions[i1], t2 = positions[i2];
        auto      n = cross(t1 - t0, t2 - t0);
        builder[i0].push_back(n);
        builder[i1].push_back(n);
        builder[i2].push_back(n);
    }

    res.resize(positions.size());
    // std::transform(builder.begin(), builder.end(), res.begin(),
    //               [](const std::vector<vec3>& c) -> glm::vec3 {
    //                   auto sum = std::accumulate(c.begin(), c.end(), vec3(0, 0, 0));
    //                   return glm::vec3(sum.normalized());
    //               });

    for (int i = 0; i < builder.size(); ++i) {
        const auto& candidates = builder[i];
        CHECK(candidates.size() > 0);
        auto sum = std::accumulate(candidates.begin(), candidates.end(), glm::vec3(0, 0, 0));
        
        res[i]   = glm::vec3(sum) / cast_f32(builder.size());
        res[i]   = glm::normalize(res[i]);
    }
    return res;
}


std::vector<glm::vec3> ComputeNormals(const std::vector<glm::vec3>& positions,
                                      const std::vector<int>&       indices)
{
    std::vector<glm::vec3>              normals;
    std::vector<std::vector<glm::vec3>> normal_builder;
    CHECK_EQ(indices.size() % 3, 0);

    normals.resize(positions.size(), glm::vec3(0, 0, 0));
    for (int i = 0; i < indices.size(); i += 3) {
        glm::vec3 p0 = positions[indices[i + 0]];
        glm::vec3 p1 = positions[indices[i + 1]];
        glm::vec3 p2 = positions[indices[i + 2]];

        auto n = glm::cross(p1 - p0, p2 - p0);
        normals[indices[i + 0]] += n;
        normals[indices[i + 1]] += n;
        normals[indices[i + 2]] += n;
    }
    return normals;
}


std::vector<std::string> split(const std::string& s, char delimiter)
{
    std::vector<std::string> res;
    size_t                   idxstart = 0, idxend = 0;
    while (idxend != std::string::npos) {
        idxend = s.find(delimiter, idxstart);
        // Empty strings (adjacent delimiter characters) don't count
        if (idxend != idxstart)
            res.push_back(s.substr(idxstart, idxend - idxstart));
        if (res.back().empty())
            res.pop_back();
        idxstart = idxend + 1;
    }
    return res;
}

bool SanityCheck(const vkpbr::PlyMesh& m)
{
    bool res = true;
    if (m.normals.empty() == false) {
        if (m.normals.size() != m.positions.size()) {
            std::cerr << "normal vectors count " << m.normals.size() << " differs from pos count "
                      << m.positions.size() << std::endl;
            res = false;
        }
    }
    if (m.texture_uvs.empty() == false && m.texture_uvs.size() != m.positions.size()) {
        std::cerr << "uv count " << m.normals.size() << " differs from pos count "
                  << m.positions.size() << std::endl;
        res = false;
    }
    for (size_t i = 0; i < m.positions.size(); ++i) {
        const glm::vec3& pos = m.positions[i];
        if (std::isnan(pos.x) || std::isnan(pos.y) || std::isnan(pos.z)) {
            std::cerr << "pos[" << i << "] has NaN: " << glm::to_string(pos) << std::endl;
            res = false;
        }
        if (!m.normals.empty()) {
            const glm::vec3& n = m.normals[i];
            if (std::isnan(n.x) || std::isnan(n.y) || std::isnan(n.z)) {
                std::cerr << "normal[" << i << "] has NaN: " << glm::to_string(n) << std::endl;
                res = false;
            }
        }
        if (!m.texture_uvs.empty()) {
            const glm::vec2& uv = m.texture_uvs[i];
            if (std::isnan(uv.x) || std::isnan(uv.y)) {
                std::cerr << "uv[" << i << "] has NaN: " << uv.x << ',' << uv.y << std::endl;
                res = false;
            }
        }
    }
    if (!m.indices.empty())
        for (size_t i = 0; i < m.indices.size(); ++i) {
            if (m.indices[i] >= m.positions.size() || m.indices[i] < 0) {
                std::cerr << "indices[" << i << "] out side of boundary: " << m.indices[i]
                          << std::endl;
                res = false;
            }
        }
    return res;
}
