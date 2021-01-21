#version 460
#extension GL_EXT_ray_tracing : require
#extension GL_GOOGLE_include_directive : enable
#include "raycommon.glsl"

layout(location = 0) rayPayloadInEXT hitPayload prd;

layout(push_constant) uniform Constants
{
    vec4 clearColor;
};

void main()
{
    if (prd.depth == 0)
        prd.hitValue = clearColor.xyz * 0.01;
    else
        prd.hitValue = clearColor.xyz * 0.01;
    prd.depth = 100;  // Ends path tracing.
}
