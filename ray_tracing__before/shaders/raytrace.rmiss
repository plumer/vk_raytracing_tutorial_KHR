#version 460
#extension GL_EXT_ray_tracing : require
#extension GL_GOOGLE_include_directive : enable
#include "raycommon.glsl"

layout(location = 0) rayPayloadInEXT HitPayload prd;

layout(push_constant) uniform Constants {
    vec4 clear_color;
};

void main()
{
//    prd.hit_value = clear_color.xyz * 0.3;
    prd.hit_value = vec3(0.2, 0.3, 0.3);
}