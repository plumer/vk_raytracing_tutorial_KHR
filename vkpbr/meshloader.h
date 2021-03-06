//
//  meshloader.h

#ifndef meshloader_h
#define meshloader_h

#include "pbrt_scene.h"
#include <glm/glm.hpp>

/**
 * Loads a mesh from a .ply file return geometry data.
 * If the file does not exist, returns an empty PlyMesh object.
 * \param enforce_triangle Splits polygons (mostly quads) into 2 triangles.
 */
vkpbr::PlyMesh LoadPly(const std::string& filename, bool enforce_triangle);

/**
 * Loads a mesh from a .obj wavefront file.
 * \param enforce_triangle Splits polygons (mostly quads) into 2 triangles.
 */
vkpbr::PlyMesh LoadObj(const std::string &filename, bool enforce_triangle);

/**
 * Loads a triangle mesh from file. Supported file formats include .obj and .ply.
 */
vkpbr::PlyMesh LoadTriangleMesh(const std::string& filename);

vkpbr::PlyMesh LoadTriangleMeshBVH(const std::string& file_name, const glm::mat4& obj_to_world,
                                      bool reverse_orientation);

std::pair<
    std::vector<vkpbr::VertexData>,
    std::vector<int>
> ToArrayOfStructures(const vkpbr::PlyMesh & ply_mesh);

/**
 * \brief Interleaves vertex attribute data into array of vkpbr::VertexData (3 + 3 + 2) format.
 * \param positions Must be non-empty.
 * \param normals Must have same length of `positions`.
 * \param texture_uvs Can be empty, in which case initialized by default glm::vec2 constructor.
 */
std::vector<vkpbr::VertexData> Interleave(const std::vector<glm::vec3>& positions,
                                          const std::vector<glm::vec3>& normals,
                                          const std::vector<glm::vec2>& texture_uvs = {});

// Computes the normals of given vertex positions and triangulation.
std::vector<glm::vec3> compute_normals(const std::vector<glm::vec3>& positions,
                                       const std::vector<int>&       tri_indices);



std::vector<glm::vec3> ComputeNormals(const std::vector<glm::vec3>& positions,
                                      const std::vector<int>&       indices);

#endif /* meshloader_h */
