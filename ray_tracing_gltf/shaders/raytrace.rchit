#version 460
#extension GL_EXT_ray_tracing : require
#extension GL_EXT_nonuniform_qualifier : enable
#extension GL_EXT_scalar_block_layout : enable
#extension GL_GOOGLE_include_directive : enable

hitAttributeEXT vec2 attribs;

#include "raycommon.glsl"
#include "bindings.glsl"
#include "gltf.glsl"
#include "sampling.glsl"

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
    // Retrieves the primitive mesh buffer information.
    PrimMeshInfo p_info = ds_prim_info[gl_InstanceCustomIndexEXT];

    // Retrieves the 'first index' for this mesh (offset of the mesh + offset of the triangle).
    uint index_offset  = p_info.indexOffset + (3 * gl_PrimitiveID);
    uint vertex_offset = p_info.vertexOffset;
    uint mtl_index     = max(0, p_info.materialIndex);

    // Retrieves the 3 ds_indices of the triangle (local).
    ivec3 triangle_index = ivec3(ds_indices[nonuniformEXT(index_offset + 0)],
                                 ds_indices[nonuniformEXT(index_offset + 1)],
                                 ds_indices[nonuniformEXT(index_offset + 2)]);
    triangle_index += ivec3(vertex_offset);  // (global)

    const vec3 barycentrics = vec3(1.0 - attribs.x - attribs.y, attribs.x, attribs.y);

    // Computes geometric information of the intersection point: position, normal, and texture UV.
    // --------------------------------------------------------------------------------------------

    // Vertex of the triangle
    const vec3 pos0     = GetVertex(triangle_index.x);
    const vec3 pos1     = GetVertex(triangle_index.y);
    const vec3 pos2     = GetVertex(triangle_index.z);
    const vec3 position = pos0 * barycentrics.x + pos1 * barycentrics.y + pos2 * barycentrics.z;

    // Computing the coordinates of the hit position
    vec3 world_position = vec3(gl_ObjectToWorldEXT * vec4(position, 1.0));

    // Computing the normal at hit position
    const vec3 norm0 = GetNormal(triangle_index.x);
    const vec3 norm1 = GetNormal(triangle_index.y);
    const vec3 norm2 = GetNormal(triangle_index.z);
    const vec3 normal =
        normalize(norm0 * barycentrics.x + norm1 * barycentrics.y + norm2 * barycentrics.z);
    // Transforming the normal to world space
    const vec3 world_normal     = normalize(vec3(normal * gl_ObjectToWorldEXT));
    const vec3 geometric_normal = normalize(cross(pos1 - pos0, pos2 - pos0));

    // Computes the texture UV coordinates at hit position.
    const vec2 uv0       = GetTexCoord(triangle_index.x);
    const vec2 uv1       = GetTexCoord(triangle_index.y);
    const vec2 uv2       = GetTexCoord(triangle_index.z);
    const vec2 texcoord0 = uv0 * barycentrics.x + uv1 * barycentrics.y + uv2 * barycentrics.z;

    // Computes the radiance along the ray.
    // --------------------------------------------------------------------------------------------
    GltfMaterial mtl       = ds_materials[nonuniformEXT(mtl_index)];
    vec3         emittance = mtl.emissiveFactor;

    // Pick a random direction from the surface intersection and keep going.
    vec3 tangent, bitangent;
    MakeCoordinateSystem(world_normal, tangent, bitangent);

    vec3 ray_origin    = world_position;
    vec3 ray_direction = mat3(tangent, bitangent, world_normal) * SampleLocalHemi(ray_payload.seed);
    
    // Probability of the new ray.
    const float kPr = 1 / M_PI;
    
    // Computes the BRDF for the ray.
    float cos_theta = dot(ray_direction, world_normal);
    vec3 albedo = mtl.pbrBaseColorFactor.xyz;
    vec3 BRDF = albedo / M_PI;
    
    ray_payload.ray_origin = ray_origin;
    ray_payload.ray_direction = ray_direction;
    ray_payload.hitValue = emittance;
    ray_payload.bsdf_weight = BRDF * cos_theta / kPr;  // !!
    
    return;
}
