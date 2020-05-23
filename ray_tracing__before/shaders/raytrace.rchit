#version 460
#extension GL_EXT_ray_tracing : require
#extension GL_EXT_nonuniform_qualifier : enable
#extension GL_EXT_scalar_block_layout  : enable
#extension GL_GOOGLE_include_directive : enable
#include "raycommon.glsl"
#include "wavefront.glsl"

layout(location = 0) rayPayloadInEXT HitPayload prd;
// The payload carried by shadow rays.
layout(location = 1) rayPayloadEXT bool is_shadowed;
hitAttributeEXT vec3 attribs;

// Used to cast shadow rays.
layout(binding = 0, set = 0)         uniform accelerationStructureEXT top_level_AS;
layout(binding = 1, set = 1, scalar) buffer MatColorBufferObject {WaveFrontMaterial m[];} materials[];
layout(binding = 2, set = 1, scalar) buffer ScnDesc {sceneDesc i[]; } scene_description;
layout(binding = 3, set = 1)         uniform sampler2D textureSamplers[];
layout(binding = 4, set = 1)         buffer MatIndexColorBuffer{ int i[]; } matIndex[];
layout(binding = 5, set = 1, scalar) buffer Vertices { Vertex v[]; } vertices[];
layout(binding = 6, set = 1)         buffer Indices { uint i[]; } indices[];

layout(push_constant) uniform Constants {
    vec4  clear_color;
    vec3  light_position;
    float light_intensity;
    int   light_type;
} push_c;

vec3 barycentric_lerp(vec3 bc_coords, vec3 x, vec3 y, vec3 z) {
    return x * bc_coords.x + y * bc_coords.y + z * bc_coords.z;
}

vec2 barycentric_lerp(vec3 bc_coords, vec2 x, vec2 y, vec2 z) {
    return x * bc_coords.x + y * bc_coords.y + z * bc_coords.z;
}

void main() {
    uint obj_id = scene_description.i[gl_InstanceID].objId;

    ivec3 indices = ivec3(indices[obj_id].i[3 * gl_PrimitiveID + 0],
                          indices[obj_id].i[3 * gl_PrimitiveID + 1],
                          indices[obj_id].i[3 * gl_PrimitiveID + 2]);
    
    // Vertices of the triangle
    Vertex v0 = vertices[obj_id].v[indices.x];
    Vertex v1 = vertices[obj_id].v[indices.y];
    Vertex v2 = vertices[obj_id].v[indices.z];

    const vec3 barycentric_coords = vec3(1.0 - attribs.x - attribs.y, attribs.x, attribs.y);
    // Interpolates the normal at hit position.
    vec3 normal = barycentric_lerp(barycentric_coords, v0.nrm, v1.nrm, v2.nrm);
    normal = normalize(vec3(scene_description.i[gl_InstanceID].transfoIT * vec4(normal, 0.0)));

    vec3 world_position = barycentric_lerp(barycentric_coords, v0.pos, v1.pos, v2.pos);
    world_position = vec3(scene_description.i[gl_InstanceID].transfo * vec4(world_position, 1.0));

    // \omega_L
    vec3 wl;
    float light_radiance = push_c.light_intensity;
    float light_distance = 100000.0;

    if (push_c.light_type == 0) {
        // Point light.
        vec3 light_direction = push_c.light_position - world_position;
        light_distance = length(light_direction);
        // Quadratic attenuation on radiance.
        light_radiance = push_c.light_intensity / (light_distance * light_distance);
        wl = normalize(light_direction);
    } else {
        // Directional light - no radiance attenuation.
        wl = normalize(push_c.light_position - vec3(0));
    }

    // Fetches the material of the object.
    // There is one buffer of materials per object and each material can be accessed
    // via the index; each triangle has an index of material.
    int mtl_index = matIndex[obj_id].i[gl_PrimitiveID];
    WaveFrontMaterial mtl = materials[obj_id].m[mtl_index];

    // Blinn-phong Diffuse + Specular components.
    vec3 diffuse = computeDiffuse(mtl, wl, normal);
    if (mtl.textureId >= 0) {
        uint texture_id = mtl.textureId + scene_description.i[gl_InstanceID].txtOffset;
        vec2 texture_uv = 
            barycentric_lerp(barycentric_coords, v0.texCoord, v1.texCoord, v2.texCoord);
        diffuse *= texture(textureSamplers[texture_id], texture_uv).xyz;
    }

    vec3 specular = vec3(0);
    float attenuation = 1;

    // Traces shadow ray only if the light is on the normal-position side of the surface.
    if (dot(normal, wl) > 0) {
        float tMin = 0.001;
        float tMax = light_distance;
        vec3 origin = gl_WorldRayOriginEXT + gl_WorldRayDirectionEXT * gl_HitTEXT;
        vec3 ray_dir = wl;
        uint flags = gl_RayFlagsTerminateOnFirstHitEXT | gl_RayFlagsOpaqueEXT |
                     gl_RayFlagsSkipClosestHitShaderEXT;
        is_shadowed = true;
        traceRayEXT(top_level_AS,
                     flags, 
                     0xFF,
                     0,         // sbt record offset
                     0,         // sbt_record stride
                     1,         // miss-shader index
                     origin, tMin, ray_dir, tMax,
                     1          // payload location
        );
        if (is_shadowed) {
            attenuation = 0.3;
            // Specular remains 0.
        } else {
            // Attenuation remains 1 - no attenuation.
            specular = computeSpecular(mtl, gl_WorldRayDirectionEXT, wl, normal);
        }
    }

    prd.hit_value = vec3(light_radiance * attenuation * (diffuse + specular));
}
