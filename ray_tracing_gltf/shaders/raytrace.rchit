#version 460
#extension GL_EXT_ray_tracing : require
#extension GL_EXT_nonuniform_qualifier : enable
#extension GL_EXT_scalar_block_layout : enable
#extension GL_GOOGLE_include_directive : enable
#include "raycommon.glsl"
#include "wavefront.glsl"

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

    int   selected_mtl_index;
    vec3  selected_color;
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

    // Vector toward the light
    vec3  L;
    float lightIntensity = pushC.lightIntensity;
    float lightDistance  = 100000.0;
    // Point light
    if (pushC.lightType == 0) {
        vec3 lDir      = pushC.lightPosition - world_position;
        lightDistance  = length(lDir);
        lightIntensity = pushC.lightIntensity / (lightDistance * lightDistance);
        L              = normalize(lDir);
    } else  // Directional light
    {
        L = normalize(pushC.lightPosition - vec3(0));
    }

    // Material of the object
    GltfMaterial mtl = ds_materials[nonuniformEXT(mtl_index)];
    // Diffuse
    vec3 diffuse = computeDiffuse(mtl, L, world_normal);
    if (mtl.pbrBaseColorTexture > -1) {
        uint txtId = mtl.pbrBaseColorTexture;
        diffuse *= texture(ds_textures[nonuniformEXT(txtId)], texcoord0).xyz;
    }

    vec3  specular    = vec3(0);
    float attenuation = 1;

    // Traces shadow ray only if the light is on the same side with the normal on the surface.
    if (dot(world_normal, L) > 0) {
        float tMin   = 0.001;
        float tMax   = lightDistance;
        vec3  origin = gl_WorldRayOriginEXT + gl_WorldRayDirectionEXT * gl_HitTEXT;
        vec3  rayDir = L;
        uint  flags  = gl_RayFlagsTerminateOnFirstHitEXT | gl_RayFlagsOpaqueEXT
                     | gl_RayFlagsSkipClosestHitShaderEXT;
        isShadowed = true;
        traceRayEXT(ds_top_level_AS,  // acceleration structure
                    flags,            // rayFlags
                    0xFF,             // cullMask
                    0,                // sbtRecordOffset
                    0,                // sbtRecordStride
                    1,                // missIndex
                    origin,           // ray origin
                    tMin,             // ray min range
                    rayDir,           // ray direction
                    tMax,             // ray max range
                    1                 // payload (location = 1)
        );

        if (isShadowed) {
            attenuation = 0.3;
        } else {
            // Specular
            specular = computeSpecular(mtl, gl_WorldRayDirectionEXT, L, normal);
        }
    }

    ray_payload.hitValue = vec3(lightIntensity * attenuation * (diffuse + specular));

    //    if (mtl.emissiveTexture < 0 && dot(mtl.emissiveFactor, vec3(1, 1, 1)) > 0.1) {
    //        ray_payload.hitValue = vec3(0.2, 0.8, 1.0);
    //    }

    vec3 kMtlIndexColorCode[10] = {vec3(0.1, 0.3, 0.7), vec3(0.3, 0.1, 0.7), vec3(0.7, 0.1, 0.3),
                                   vec3(0.7, 0.3, 0.1), vec3(0.1, 0.7, 0.3), vec3(0.3, 0.7, 0.1),
                                   vec3(0.3, 0.3, 0.7), vec3(0.7, 0.7, 0.1), vec3(0.1, 0.1, 0.1),
                                   vec3(0.7, 0.7, 0.7)};
    if (pushC.selected_mtl_index == mtl_index) {
        ray_payload.hitValue = kMtlIndexColorCode[mtl_index];
        if (mtl.emissiveFactor.x > 0.01)
            ray_payload.hitValue = vec3(0.8, 0.5, 0.1);
    }
}
