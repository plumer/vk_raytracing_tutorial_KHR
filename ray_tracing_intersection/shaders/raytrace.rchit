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
layout(location = 0) rayPayloadInEXT hitPayload prd;
layout(location = 1) rayPayloadEXT bool isShadowed;

layout(binding = 0, set = 0) uniform accelerationStructureEXT topLevelAS;
layout(binding = 2, set = 0) readonly buffer _InstanceInfo {PrimMeshInfo prim_info[];};

// layout(binding = 2, set = 1, scalar) buffer ScnDesc { sceneDesc i[]; } scnDesc;
// layout(binding = 5, set = 1, scalar) buffer Vertices { Vertex v[]; } vertices[];
// layout(binding = 6, set = 1) buffer Indices { uint i[]; } indices[];

// layout(binding = 1, set = 1, scalar) buffer MatColorBufferObject { WaveFrontMaterial m[]; } materials[];
// layout(binding = 3, set = 1) uniform sampler2D textureSamplers[];
// layout(binding = 4, set = 1)  buffer MatIndexColorBuffer { int i[]; } matIndex[];

layout(set = 1, binding = kDsbVertices) readonly buffer _VertexBuf {float vertices[];};
layout(set = 1, binding = kDsbIndices) readonly buffer _Indices {uint indices[];};
layout(set = 1, binding = kDsbNormals) readonly buffer _NormalBuf {float normals[];};
layout(set = 1, binding = kDsbTexcoords) readonly buffer _TexCoordBuf {float texcoord0[];};
layout(set = 1, binding = kDsbMaterials) readonly buffer _MaterialBuffer {GltfMaterial materials[];};
layout(set = 1, binding = kDsbTextures) uniform sampler2D texturesMap[];

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
    return vec3(vertices[3 * index + 0], vertices[3 * index + 1], vertices[3 * index + 2]);
}

vec3 GetNormal(uint index)
{
    return vec3(normals[3 * index + 0], normals[3 * index + 1], normals[3 * index + 2]);
}

vec2 GetTexCoord(uint index)
{
    return vec2(texcoord0[2 * index + 0], texcoord0[2 * index + 1]);
}

void main()
{
    // Retrieves the primitive mesh buffer information.
    PrimMeshInfo p_info = prim_info[gl_InstanceCustomIndexEXT];

    // Retrieves the 'first index' for this mesh (offset of the mesh + offset of the triangle).
    uint index_offset  = p_info.indexOffset + (3 * gl_PrimitiveID);
    uint vertex_offset = p_info.vertexOffset;
    uint mtl_index     = max(0, p_info.materialIndex);

    // Retrieves the 3 indices of the triangle (local).
    ivec3 triangle_index =
        ivec3(indices[nonuniformEXT(index_offset + 0)], indices[nonuniformEXT(index_offset + 1)],
              indices[nonuniformEXT(index_offset + 2)]);
    triangle_index += ivec3(vertex_offset);  // (global)

    const vec3 barycentrics = vec3(1.0 - attribs.x - attribs.y, attribs.x, attribs.y);

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
    const vec2 uv0 = GetTexCoord(triangle_index.x);
    const vec2 uv1 = GetTexCoord(triangle_index.y);
    const vec2 uv2 = GetTexCoord(triangle_index.z);
    const vec2 texcoord0 = uv0 * barycentrics.x + uv1 * barycentrics.y + uv2 * barycentrics.z;

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
    GltfMaterial mtl = materials[nonuniformEXT(mtl_index)];


    // Diffuse
    vec3 diffuse = computeDiffuse(mtl, L, world_normal);
    if (mtl.pbrBaseColorTexture > -1) {
        uint txtId    = mtl.pbrBaseColorTexture;
        diffuse *= texture(texturesMap[nonuniformEXT(txtId)], texcoord0).xyz;
    }

    vec3  specular    = vec3(0);
    float attenuation = 1;

    // Tracing shadow ray only if the light is visible from the surface
    if (dot(world_normal, L) > 0) {
        float tMin   = 0.001;
        float tMax   = lightDistance;
        vec3  origin = gl_WorldRayOriginEXT + gl_WorldRayDirectionEXT * gl_HitTEXT;
        vec3  rayDir = L;
        uint  flags  = gl_RayFlagsTerminateOnFirstHitEXT | gl_RayFlagsOpaqueEXT
                     | gl_RayFlagsSkipClosestHitShaderEXT;
        isShadowed = true;
        traceRayEXT(topLevelAS,  // acceleration structure
                    flags,       // rayFlags
                    0xFF,        // cullMask
                    0,           // sbtRecordOffset
                    0,           // sbtRecordStride
                    1,           // missIndex
                    origin,      // ray origin
                    tMin,        // ray min range
                    rayDir,      // ray direction
                    tMax,        // ray max range
                    1            // payload (location = 1)
        );

        if (isShadowed) {
            attenuation = 0.3;
        } else {
            // Specular
            specular = computeSpecular(mtl, gl_WorldRayDirectionEXT, L, normal);
        }
    }

    prd.hitValue = vec3(lightIntensity * attenuation * (diffuse + specular));
}
