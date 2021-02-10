#version 460
#extension GL_EXT_ray_tracing : require
#extension GL_EXT_nonuniform_qualifier : enable
#extension GL_EXT_scalar_block_layout : enable
#extension GL_GOOGLE_include_directive : enable
#include "raycommon.glsl"
#include "sampling.glsl"
#include "wavefront.glsl"

hitAttributeEXT vec2 attribs;

// clang-format off
layout(location = 0) rayPayloadInEXT hitPayload payload_in;
layout(location = 1) rayPayloadEXT bool isShadowed;

// Ray-tracing descriptors
// ------------------------------------------------------------------------------------------------
layout(binding = 0, set = 0) uniform accelerationStructureEXT topLevelAS;

// Regular descriptors
// ------------------------------------------------------------------------------------------------
layout(binding = 1, set = 1, scalar) buffer PbrtMaterialArray {PbrtMaterial mtls[];} p_mtls;

// Stores information for all object instances (ID, transform, etc) in the scene.
// Can be indexed by gl_InstanceID.
layout(binding = 2, set = 1, scalar) buffer ScnDesc { InstanceInfo instances[]; } scene;
   
layout(binding = 3, set = 1) uniform sampler2D textureSamplers[];

// Stores vertex data for the object mesh, for all objects.
// `v` can be indexed through `indices` at DS binding 6.
layout(binding = 5, set = 1, scalar) buffer Vertices { Vertex v[]; } vertices[];

// Triangle indices to vertex data for all object model.
layout(binding = 6, set = 1) buffer Indices { uint i[]; } indices[];

layout(binding = 7, set = 1, scalar) buffer UniversalMaterials{ WaveFrontMaterial mtls[];} u_mtls;

layout(push_constant) uniform Constants
{
    vec4  clearColor;
    vec3  lightPosition;
    float lightIntensity;
    int   lightType;
    int   accumulated_frames;
    int   max_recursion_depth;
    float glass_ior;
}
pushC;

// Built-in variables used:
// ------------------------------------------------------------------------------------------------

// gl_InstanceID is available in the intersection, any-hit, and closest-hit
// shaders to specify the index of the _instance_ that intersects the current ray. 
// The gl_InstanceID is the index of the intersected instance as it appeared in the array of
// instances used to build the TLAS.

// nonuniformEXT - added _keyword_ to the GLSL language
// This is required by the Vulkan API to be used when indexing descriptor
// bindings with an index that is not dynamically uniform.

// gl_PrimitiveID - intersection, any-hit and closest-hit shaders
// Specifies the index of the triangle or bounding box being processed.

void main()
{
    // Obtains geometric information of the hit point: position, normal, incident light direction.
    // -------------------------------------------------------------------------------------------
    // Object of this instance
    uint objId = scene.instances[gl_InstanceID].objId;

    // Indices of the triangle
    ivec3 ind = ivec3(indices[nonuniformEXT(objId)].i[3 * gl_PrimitiveID + 0],   //
                      indices[nonuniformEXT(objId)].i[3 * gl_PrimitiveID + 1],   //
                      indices[nonuniformEXT(objId)].i[3 * gl_PrimitiveID + 2]);  //
    // Vertex of the triangle
    Vertex v0 = vertices[nonuniformEXT(objId)].v[ind.x];
    Vertex v1 = vertices[nonuniformEXT(objId)].v[ind.y];
    Vertex v2 = vertices[nonuniformEXT(objId)].v[ind.z];

    const vec3 barycentrics = vec3(1.0 - attribs.x - attribs.y, attribs.x, attribs.y);

    // Computing the normal at hit position
    vec3 normal = v0.nrm * barycentrics.x + v1.nrm * barycentrics.y + v2.nrm * barycentrics.z;
    // Transforming the normal to world space
    normal = normalize(vec3(scene.instances[gl_InstanceID].transfoIT * vec4(normal, 0.0)));

    // Computing the coordinates of the hit position
    vec3 worldPos = v0.pos * barycentrics.x + v1.pos * barycentrics.y + v2.pos * barycentrics.z;
    // Transforming the position to world space
    worldPos = vec3(scene.instances[gl_InstanceID].transfo * vec4(worldPos, 1.0));

    // Vector toward the light
    vec3  L;
    float lightIntensity = pushC.lightIntensity;
    float lightDistance  = 100000.0;
    // Point light
    if (pushC.lightType == 0) {
        vec3 lDir      = pushC.lightPosition - worldPos;
        lightDistance  = length(lDir);
        lightIntensity = pushC.lightIntensity / (lightDistance * lightDistance);
        L              = normalize(lDir);
    } else  // Directional light
    {
        L = normalize(pushC.lightPosition - vec3(0));
    }

    // Computes the integration of rendering equation.
    // -------------------------------------------------------------------------------------------
    // Material of the object
    int matIdx = scene.instances[gl_InstanceID].mtl_index;
    WaveFrontMaterial mat = u_mtls.mtls[nonuniformEXT(matIdx)];

    PbrtMaterial p_mat = p_mtls.mtls[nonuniformEXT(matIdx)];

    // Determines the light to scatter according to material.

    vec3 tangent, bitangent;
    createCoordinateSystem(normal, /*out*/ tangent, /*out*/ bitangent);
    vec3 wi_world = samplingHemisphere(/*inout*/ payload_in.seed, tangent, bitangent, normal);

    if (dot(wi_world, normal) < 0) wi_world = -wi_world;

    const float lambertian_pdf = 1.0 / 3.1415926;
    float       cos_theta      = dot(wi_world, normal);
    vec3        BRDF           = p_mat.diffuse / M_PI;


    payload_in.ray_origin    = worldPos;
    payload_in.ray_direction = wi_world;
    payload_in.hitValue      = p_mat.emission;
    payload_in.weight        = BRDF * cos_theta / lambertian_pdf;
    return;
}
