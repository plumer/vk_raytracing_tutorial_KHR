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

std::pair<std::vector<vkpbr::VertexData>, std::vector<int>> ToArrayOfStructures(
    const vkpbr::PlyMesh & ply_mesh)
{
    return std::make_pair(Interleave(ply_mesh.positions, ply_mesh.normals, ply_mesh.texture_uvs),
                          ply_mesh.indices);
}

std::vector<vkpbr::VertexData> Interleave(const std::vector<glm::vec3>& positions,
                                          const std::vector<glm::vec3>& normals,
                                          const std::vector<glm::vec2>& texture_uvs)
{
    CHECK_EQ(positions.size(), normals.size());

    std::vector<vkpbr::VertexData> vertex_attributes;
    for (int i = 0; i < positions.size(); ++i) {
        vkpbr::VertexData v;
        v.position = positions[i];
        v.normal   = normals[i];
        if (!texture_uvs.empty())
            v.texture_uv = texture_uvs[i];

        vertex_attributes.push_back(v);
    }
    return vertex_attributes;
}

std::vector<std::string> split(const std::string& s, char delimiter);

vkpbr::PlyMesh LoadObj(const std::string& filename, bool enforce_triangle)
{
    std::vector<glm::vec3> positions;
    std::vector<glm::vec3> normals;
    std::vector<glm::vec2> texture_uvs;
    std::vector<int>       position_indices;
    std::vector<int>       normal_indices;
    std::vector<int>       uv_indices;

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
            double x = std::atof(words[1].c_str());
            double y = std::atof(words[2].c_str());
            double z = std::atof(words[3].c_str());
            positions.push_back(glm::vec3{cast_f32(x), cast_f32(y), cast_f32(z)});
        } else if (words[0] == "vn") {
            CHECK_EQ(words.size(), 4);
            double x = std::atof(words[1].c_str());
            double y = std::atof(words[2].c_str());
            double z = std::atof(words[3].c_str());
            normals.push_back(glm::vec3{cast_f32(x), cast_f32(y), cast_f32(z)});
        } else if (words[0] == "vt") {
            CHECK_GE(words.size(), 3); // Some .obj files have 3 components.
            double x = std::atof(words[1].c_str());
            double y = std::atof(words[2].c_str());
            texture_uvs.push_back(glm::vec2{cast_f32(x), cast_f32(y)});
        } else if (words[0] == "f") {
            std::vector<int> face_indices, face_uv_indices, face_normal_indices;
            face_indices.reserve(words.size() - 1);
            for (size_t i = 1; i < words.size(); ++i) {
                auto index_group = split(words[i], '/');
                CHECK(!index_group.empty());
                int index = std::atoi(index_group[0].c_str());
                
                face_indices.push_back(index);
                if (index_group.size() > 1 && !index_group[1].empty())
                    face_uv_indices.push_back(std::atoi(index_group[1].c_str()));
                if (index_group.size() > 2 && !index_group[2].empty())
                    face_normal_indices.push_back(std::atoi(index_group[2].c_str()));
            }
            if (enforce_triangle && face_indices.size() > 3) {
                // If the face specifies more than 3 vertices, then tesselates the polygon.
                // Assuming the polygon is convex.
                decltype(face_indices) triangle_indices;
                triangle_indices.reserve((face_indices.size() - 2) * 3);
                // if there are 6 vertices, then the first triangle is [0, 1, 2], the last is [0, 4,
                // 5].
                for (size_t i = 1; i < face_indices.size() - 1; ++i) {
                    triangle_indices.push_back(face_indices[0]);
                    triangle_indices.push_back(face_indices[i]);
                    triangle_indices.push_back(face_indices[i + 1]);
                }
                std::swap(triangle_indices, face_indices);
            }
            LOG_IF(FATAL, face_indices.size() > 3) << "More than 3 vertices per 'f' line is not "
                                                      "supported yet";
            if (!face_uv_indices.empty())
                CHECK_EQ(face_indices.size(), face_uv_indices.size());
            if (!face_normal_indices.empty())
                CHECK_EQ(face_indices.size(), face_normal_indices.size());

            // Appends the indices of the current face to the whole set of indices.
            position_indices.insert(position_indices.end(), face_indices.begin(),
                                    face_indices.end());
            normal_indices.insert(normal_indices.end(), face_normal_indices.begin(),
                                  face_normal_indices.end());
            uv_indices.insert(uv_indices.end(), face_uv_indices.begin(), face_uv_indices.end());
        } else {
            std::cerr << "unhandled line starting with \'" << words[0] << "\'" << std::endl;
        }
    }
    if (!normal_indices.empty())
        CHECK_EQ(position_indices.size(), normal_indices.size());
    if (!uv_indices.empty())
        CHECK_EQ(position_indices.size(), uv_indices.size());
    int num_points = cast_i32(positions.size());
    for (std::vector<int>* indices_array : {&position_indices, &normal_indices, &uv_indices})
        for (int& index : *indices_array) {
            if (index < 0)
                index += num_points;  // [-nPoints, -1]
            else
                index--;  // Indices in .obj files starts from 1, so decrement it by 1.
        }

    // If the non-empty normal/uv indices aren't identical to the position indices, we'll have to
    // re-index the data. For example, if the .obj file specify things like this:
    //
    //     f 30/30/30  31/31/31  35/35/35
    // 
    // all across the entire file (every triplet has the 3 same indices), then the indexing is
    // uniform and there's no need for re-indexing. Otherwise (a rather obsolete file structure for
    // .obj files), geometry data arrays (position/uv/normal) are replaced by indexed values.
    bool nonuniform_indexing = false;
    if (!normal_indices.empty())
        nonuniform_indexing |= (normal_indices != position_indices);
    if (!uv_indices.empty())
        nonuniform_indexing |= (uv_indices != position_indices);

    if (nonuniform_indexing) {
        decltype(positions) reindexed_positions;

        decltype(texture_uvs) reindexed_uvs;
        decltype(normals) reindexed_normals;

        decltype(position_indices) identity_indices;

        for (size_t i = 0; i < position_indices.size(); ++i) {
            reindexed_positions.push_back(positions[position_indices[i]]);
            if (!uv_indices.empty())
                reindexed_uvs.push_back(texture_uvs[uv_indices[i]]);
            if (!normal_indices.empty())
                reindexed_normals.push_back(normals[normal_indices[i]]);
            identity_indices.push_back(cast_i32(i));
        }

        std::swap(reindexed_normals, normals);
        std::swap(reindexed_uvs, texture_uvs);
        std::swap(reindexed_positions, positions);
        std::swap(identity_indices, position_indices);
    }

    vkpbr::PlyMesh res;
    res.positions   = std::move(positions);
    res.texture_uvs = std::move(texture_uvs);
    res.normals     = std::move(normals);
    res.indices     = std::move(position_indices);
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
    size_t  stride    = vertex_attrib_names.size();
    //auto van_begin = vertex_attrib_names.cbegin(), van_end = vertex_attrib_names.cend();
    int  offset_px = -1, offset_py = -1, offset_pz = -1, offset_nx = -1, offset_ny = -1,
        offset_nz = -1, offset_u = -1, offset_v = -1;
    for (size_t i = 0; i < stride; ++i) {
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
    for (size_t base = 0; base < vertex_buffer.size(); base += stride) {
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
    const size_t              num_triangles = tri_indices.size() / 3;
    CHECK_EQ(num_triangles * 3, tri_indices.size());
    std::vector<std::vector<glm::vec3>> builder;
    builder.resize(positions.size());
    for (size_t tri_i = 0; tri_i < num_triangles; ++tri_i) {
        int i0 = tri_indices[tri_i * 3 + 0], i1 = tri_indices[tri_i * 3 + 1],
            i2       = tri_indices[tri_i * 3 + 2];
        glm::vec3 t0 = positions[i0], t1 = positions[i1], t2 = positions[i2];
        auto      n = cross(t1 - t0, t2 - t0);
        builder[i0].push_back(n);
        builder[i1].push_back(n);
        builder[i2].push_back(n);
    }

    res.resize(positions.size());

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
    CHECK_EQ(indices.size() % 3, 0);

    normals.resize(positions.size(), glm::vec3(0, 0, 0));
    for (size_t i = 0; i < indices.size(); i += 3) {
        glm::vec3 p0 = positions[indices[i + 0]];
        glm::vec3 p1 = positions[indices[i + 1]];
        glm::vec3 p2 = positions[indices[i + 2]];

        auto n = glm::cross(p1 - p0, p2 - p0);
        normals[indices[i + 0]] += n;
        normals[indices[i + 1]] += n;
        normals[indices[i + 2]] += n;
    }
    for (auto& n : normals) {
        n = glm::normalize(n);
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
