#version 460
#extension GL_EXT_ray_tracing : require
#extension GL_EXT_nonuniform_qualifier : enable
#extension GL_EXT_scalar_block_layout : enable
#extension GL_GOOGLE_include_directive : enable

<<<<<<< HEAD
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
=======
#include "binding.glsl"
#include "gltf.glsl"
#include "raycommon.glsl"

hitAttributeEXT vec2 attribs;

// clang-format off
layout(location = 0) rayPayloadInEXT hitPayload prd;
layout(location = 1) rayPayloadEXT bool isShadowed;

layout(set = 0, binding = 0 ) uniform accelerationStructureEXT topLevelAS;
layout(set = 0, binding = 2) readonly buffer _InstanceInfo {PrimMeshInfo primInfo[];};

layout(set = 1, binding = B_VERTICES) readonly buffer _VertexBuf {float vertices[];};
layout(set = 1, binding = B_INDICES) readonly buffer _Indices {uint indices[];};
layout(set = 1, binding = B_NORMALS) readonly buffer _NormalBuf {float normals[];};
layout(set = 1, binding = B_TEXCOORDS) readonly buffer _TexCoordBuf {float texcoord0[];};
layout(set = 1, binding = B_MATERIALS) readonly buffer _MaterialBuffer {GltfMaterial materials[];};
layout(set = 1, binding = B_TEXTURES) uniform sampler2D texturesMap[]; // all textures

>>>>>>> b7ba7d4fdc4128cefb7ab1d6097cc6d21c37d817

// clang-format on

layout(push_constant) uniform Constants
{
<<<<<<< HEAD
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
=======
  vec4  clearColor;
  vec3  lightPosition;
  float lightIntensity;
  int   lightType;
}
pushC;

// Return the vertex position
vec3 getVertex(uint index)
{
  vec3 vp;
  vp.x = vertices[3 * index + 0];
  vp.y = vertices[3 * index + 1];
  vp.z = vertices[3 * index + 2];
  return vp;
}

vec3 getNormal(uint index)
{
  vec3 vp;
  vp.x = normals[3 * index + 0];
  vp.y = normals[3 * index + 1];
  vp.z = normals[3 * index + 2];
  return vp;
}

vec2 getTexCoord(uint index)
{
  vec2 vp;
  vp.x = texcoord0[2 * index + 0];
  vp.y = texcoord0[2 * index + 1];
  return vp;
}


void main()
{
  // Retrieve the Primitive mesh buffer information
  PrimMeshInfo pinfo = primInfo[gl_InstanceCustomIndexEXT];

  // Getting the 'first index' for this mesh (offset of the mesh + offset of the triangle)
  uint indexOffset  = pinfo.indexOffset + (3 * gl_PrimitiveID);
  uint vertexOffset = pinfo.vertexOffset;           // Vertex offset as defined in glTF
  uint matIndex     = max(0, pinfo.materialIndex);  // material of primitive mesh

  // Getting the 3 indices of the triangle (local)
  ivec3 triangleIndex = ivec3(indices[nonuniformEXT(indexOffset + 0)],  //
                              indices[nonuniformEXT(indexOffset + 1)],  //
                              indices[nonuniformEXT(indexOffset + 2)]);
  triangleIndex += ivec3(vertexOffset);  // (global)

  const vec3 barycentrics = vec3(1.0 - attribs.x - attribs.y, attribs.x, attribs.y);

  // Vertex of the triangle
  const vec3 pos0           = getVertex(triangleIndex.x);
  const vec3 pos1           = getVertex(triangleIndex.y);
  const vec3 pos2           = getVertex(triangleIndex.z);
  const vec3 position       = pos0 * barycentrics.x + pos1 * barycentrics.y + pos2 * barycentrics.z;
  const vec3 world_position = vec3(gl_ObjectToWorldEXT * vec4(position, 1.0));

  // Normal
  const vec3 nrm0 = getNormal(triangleIndex.x);
  const vec3 nrm1 = getNormal(triangleIndex.y);
  const vec3 nrm2 = getNormal(triangleIndex.z);
  vec3 normal = normalize(nrm0 * barycentrics.x + nrm1 * barycentrics.y + nrm2 * barycentrics.z);
  const vec3 world_normal = normalize(vec3(normal * gl_WorldToObjectEXT));
  const vec3 geom_normal  = normalize(cross(pos1 - pos0, pos2 - pos0));

  // TexCoord
  const vec2 uv0       = getTexCoord(triangleIndex.x);
  const vec2 uv1       = getTexCoord(triangleIndex.y);
  const vec2 uv2       = getTexCoord(triangleIndex.z);
  const vec2 texcoord0 = uv0 * barycentrics.x + uv1 * barycentrics.y + uv2 * barycentrics.z;

  // Vector toward the light
  vec3  L;
  float lightIntensity = pushC.lightIntensity;
  float lightDistance  = 100000.0;
  // Point light
  if(pushC.lightType == 0)
  {
    vec3 lDir      = pushC.lightPosition - world_position;
    lightDistance  = length(lDir);
    lightIntensity = pushC.lightIntensity / (lightDistance * lightDistance);
    L              = normalize(lDir);
  }
  else  // Directional light
  {
    L = normalize(pushC.lightPosition - vec3(0));
  }

  // Material of the object
  GltfMaterial mat = materials[nonuniformEXT(matIndex)];

  // Diffuse
  vec3 diffuse = computeDiffuse(mat, L, world_normal);
  if(mat.pbrBaseColorTexture > -1)
  {
    uint txtId = mat.pbrBaseColorTexture;
    diffuse *= texture(texturesMap[nonuniformEXT(txtId)], texcoord0).xyz;
  }

  vec3  specular    = vec3(0);
  float attenuation = 1;

  // Tracing shadow ray only if the light is visible from the surface
  if(dot(world_normal, L) > 0)
  {
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

    if(isShadowed)
    {
      attenuation = 0.3;
    }
    else
    {
      // Specular
      specular = computeSpecular(mat, gl_WorldRayDirectionEXT, L, world_normal);
    }
  }

  prd.hitValue = vec3(lightIntensity * attenuation * (diffuse + specular));
>>>>>>> b7ba7d4fdc4128cefb7ab1d6097cc6d21c37d817
}
