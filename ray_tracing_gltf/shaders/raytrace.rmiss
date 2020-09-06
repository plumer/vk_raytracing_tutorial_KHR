#version 460
#extension GL_EXT_ray_tracing : require
#extension GL_GOOGLE_include_directive : enable
#include "raycommon.glsl"

layout(location = 0) rayPayloadInEXT hitPayload ray_payload;

layout(push_constant) uniform Constants
{
  vec4 clearColor;
};

void main()
{
  ray_payload.hitValue = clearColor.xyz * 0.8;
}
