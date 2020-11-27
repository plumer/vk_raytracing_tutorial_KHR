#version 460
#extension GL_EXT_ray_tracing : require
#extension GL_GOOGLE_include_directive : enable
#include "raycommon.glsl"

<<<<<<< HEAD
layout(location = 0) rayPayloadInEXT hitPayload ray_payload;

layout(push_constant) uniform Constants
{
    vec4 clearColor;
=======
layout(location = 0) rayPayloadInEXT hitPayload prd;

layout(push_constant) uniform Constants
{
  vec4 clearColor;
>>>>>>> b7ba7d4fdc4128cefb7ab1d6097cc6d21c37d817
};

void main()
{
<<<<<<< HEAD
    if (ray_payload.depth == 0)
        ray_payload.hitValue = clearColor.xyz * 0.8;
    else
        ray_payload.hitValue = vec3(0.01);  // No contribution from environment
    ray_payload.depth = 100;
=======
  prd.hitValue = clearColor.xyz * 0.8;
>>>>>>> b7ba7d4fdc4128cefb7ab1d6097cc6d21c37d817
}
