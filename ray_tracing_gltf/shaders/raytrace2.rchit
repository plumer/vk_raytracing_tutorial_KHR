#version 460
#extension GL_EXT_ray_tracing : require
#extension GL_EXT_nonuniform_qualifier : enable
#extension GL_EXT_scalar_block_layout : enable
#extension GL_GOOGLE_include_directive : enable
#include "raycommon.glsl"
#include "wavefront.glsl"

// raytrace2.rchit
//
// Closest-Hit shader implementing lambertian reflective materials.
// If the ray hits an emissive material, returns the radiance in the hitPayload and sets
// `hits_emissive` a non-zero value. Otherwise, sets 0 in `hits_emissive` and computes a reflected
// ray origin and direction.

hitAttributeEXT vec2 attribs;

#include "bindings.glsl"
#include "gltf.glsl"

// clang-format off

// rayPayloadInEXT specifies an output variable. In this case, the variable ray_payload is supposed
// to be written before main() ends.
layout(location = 0) rayPayloadInEXT hitPayload ray_payload;
layout(location = 1) rayPayloadEXT bool isShadowed;

layout(binding = 0, set = 0) uniform accelerationStructureEXT ds_top_level_AS;
layout(binding = 2, set = 0) readonly buffer _InstanceInfo {PrimMeshInfo ds_prim_info[];};

layout(set = 1, binding = kDsbVertices) readonly buffer _VertexBuf {float ds_vertices[];};
layout(set = 1, binding = kDsbIndices) readonly buffer _Indices {uint ds_indices[];};
layout(set = 1, binding = kDsbNormals) readonly buffer _NormalBuf {float ds_normals[];};
layout(set = 1, binding = kDsbTexcoords) readonly buffer _TexCoordBuf {float ds_texcoord0[];};
layout(set = 1, binding = kDsbMaterials) readonly buffer _MaterialBuffer {GltfMaterial ds_materials[];};
layout(set = 1, binding = kDsbTextures) uniform sampler2D ds_textures[];

// clang-format on

layout(push_constant) uniform Constants
{
    vec4  clearColor;
    vec3  lightPosition;
    float lightIntensity;
    int   lightType;
}
pushC;

// Retrieves the vertex position.
vec3 GetVertex(uint index)
{
    return vec3(ds_vertices[3 * index + 0], ds_vertices[3 * index + 1], ds_vertices[3 * index + 2]);
}

vec3 GetNormal(uint index)
{
    return vec3(ds_normals[3 * index + 0], ds_normals[3 * index + 1], ds_normals[3 * index + 2]);
}

vec2 GetTexCoord(uint index)
{
    return vec2(ds_texcoord0[2 * index + 0], ds_texcoord0[2 * index + 1]);
}

void main()
{
    // Retrives the primitive mesh buffer information. glInstanceCustomIndexEXT is associated with
    // vkpbr::RaytracingBuilderKHR::Instance::instanceId and is prepared in HelloVulkan::
    // createTopLevelA().
    PrimMeshInfo p_info = ds_prim_info[gl_InstanceCustomIndexEXT];

    uint index_offset  = p_info.indexOffset + (3 * gl_PrimitiveID);
    uint vertex_offset = p_info.vertexOffset;
    uint mtl_index     = max(0, p_info.materialIndex);

    // Retrieves the 3 indices of the triangle.
    ivec3 triangle_indices = ivec3(ds_indices[nonuniformEXT(index_offset + 0)],
                                   ds_indices[nonuniformEXT(index_offset + 1)],
                                   ds_indices[nonuniformEXT(index_offset + 2)]);
    triangle_indices += ivec3(vertex_offset);

    const vec3 barycentrics = vec3(1.0 - attribs.x - attribs.y, attribs.x, attribs.y);

    // Vertex of the triangle
    const vec3 pos0     = GetVertex(triangle_indices.x);
    const vec3 pos1     = GetVertex(triangle_indices.y);
    const vec3 pos2     = GetVertex(triangle_indices.z);
    const vec3 position = BarycentricInterpolate(barycentrics, pos0, pos1, pos2);

    // Computing the coordinates of the hit position
    vec3 world_position = vec3(gl_ObjectToWorldEXT * vec4(position, 1.0));

    // Computing the normal at hit position
    const vec3 norm0  = GetNormal(triangle_indices.x);
    const vec3 norm1  = GetNormal(triangle_indices.y);
    const vec3 norm2  = GetNormal(triangle_indices.z);
    const vec3 normal = normalize(BarycentricInterpolate(barycentrics, norm0, norm1, norm2));
    // Transforming the normal to world space
    const vec3 world_normal     = normalize(vec3(normal * gl_ObjectToWorldEXT));
    const vec3 geometric_normal = normalize(cross(pos1 - pos0, pos2 - pos0));

    // Computes the texture UV coordinates at hit position.
    const vec2 uv0       = GetTexCoord(triangle_indices.x);
    const vec2 uv1       = GetTexCoord(triangle_indices.y);
    const vec2 uv2       = GetTexCoord(triangle_indices.z);
    const vec2 texcoord0 = BarycentricInterpolate(barycentrics, uv0, uv1, uv2);


    GltfMaterial mtl = ds_materials[nonuniformEXT(mtl_index)];
    if (mtl.emissiveTexture < 0) {
        ray_payload.hitValue = vec3(0, 0, 0);
    }
}