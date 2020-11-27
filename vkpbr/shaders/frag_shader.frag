#version 450
#extension GL_ARB_separate_shader_objects : enable
#extension GL_EXT_nonuniform_qualifier : enable
#extension GL_GOOGLE_include_directive : enable
#extension GL_EXT_scalar_block_layout : enable

<<<<<<< HEAD
#include "bindings.glsl"
#include "gltf.glsl"
#include "wavefront.glsl"

layout(push_constant) uniform shaderInformation
{
    vec3  lightPosition;
    uint  instanceId;
    float lightIntensity;
    int   lightType;
    int   materialId;
=======
#include "binding.glsl"
#include "gltf.glsl"


layout(push_constant) uniform shaderInformation
{
  vec3  lightPosition;
  uint  instanceId;
  float lightIntensity;
  int   lightType;
  int   matetrialId;
>>>>>>> b7ba7d4fdc4128cefb7ab1d6097cc6d21c37d817
}
pushC;

// clang-format off
// Incoming 
//layout(location = 0) flat in int matIndex;
layout(location = 1) in vec2 fragTexCoord;
layout(location = 2) in vec3 fragNormal;
layout(location = 3) in vec3 viewDir;
layout(location = 4) in vec3 worldPos;
// Outgoing
layout(location = 0) out vec4 outColor;
<<<<<<< HEAD
// // Buffers
// layout(binding = 1, scalar) buffer MatColorBufferObject { WaveFrontMaterial m[]; } materials[];
// layout(binding = 2, scalar) buffer ScnDesc { sceneDesc i[]; } scnDesc;
// layout(binding = 3) uniform sampler2D[] textureSamplers;
// layout(binding = 4, scalar) buffer MatIndex { int i[]; } matIdx[];
layout(binding = kDsbMaterials, set = 0) buffer _GltfMaterial {
  GltfMaterial materials[];
  };
layout(binding = kDsbTextures, set = 0) uniform sampler2D[] textureSamplers;
=======
// Buffers
layout(set = 0, binding = B_MATERIALS) buffer _GltfMaterial { GltfMaterial materials[]; };
layout(set = 0, binding = B_TEXTURES) uniform sampler2D[] textureSamplers;
>>>>>>> b7ba7d4fdc4128cefb7ab1d6097cc6d21c37d817

// clang-format on


void main()
{
<<<<<<< HEAD
    // // Object of this instance
    // int objId = scnDesc.i[pushC.instanceId].objId;

    // // Material of the object
    // int               matIndex = matIdx[objId].i[gl_PrimitiveID];
    // WaveFrontMaterial mat      = materials[objId].m[matIndex];

    GltfMaterial mat = materials[nonuniformEXT(pushC.materialId)];

    vec3 N = normalize(fragNormal);

    // Vector toward light
    vec3  L;
    float lightIntensity = pushC.lightIntensity;
    if (pushC.lightType == 0) {
        vec3  lDir     = pushC.lightPosition - worldPos;
        float d        = length(lDir);
        lightIntensity = pushC.lightIntensity / (d * d);
        L              = normalize(lDir);
    } else {
        L = normalize(pushC.lightPosition - vec3(0));
    }


    // Diffuse
    vec3 diffuse = computeDiffuse(mat, L, N);
    // if(mat.textureId >= 0)
    // {
    //   int  txtOffset  = scnDesc.i[pushC.instanceId].txtOffset;
    //   uint txtId      = txtOffset + mat.textureId;
    //   vec3 diffuseTxt = texture(textureSamplers[txtId], fragTexCoord).xyz;
    //   diffuse *= diffuseTxt;
    // }
    if (mat.pbrBaseColorTexture > -1) {
        uint texture_id = mat.pbrBaseColorTexture;
        vec3 diffuse_texture =
            texture(textureSamplers[nonuniformEXT(texture_id)], fragTexCoord).xyz;
        diffuse *= diffuse_texture;
    }

    // Specular
    vec3 specular = computeSpecular(mat, viewDir, L, N);

    // Result
    outColor = vec4(lightIntensity * (diffuse + specular), 1);
=======
  // Material of the object
  GltfMaterial mat = materials[nonuniformEXT(pushC.matetrialId)];

  vec3 N = normalize(fragNormal);

  // Vector toward light
  vec3  L;
  float lightIntensity = pushC.lightIntensity;
  if(pushC.lightType == 0)
  {
    vec3  lDir     = pushC.lightPosition - worldPos;
    float d        = length(lDir);
    lightIntensity = pushC.lightIntensity / (d * d);
    L              = normalize(lDir);
  }
  else
  {
    L = normalize(pushC.lightPosition - vec3(0));
  }


  // Diffuse
  vec3 diffuse = computeDiffuse(mat, L, N);
  if(mat.pbrBaseColorTexture > -1)
  {
    uint txtId      = mat.pbrBaseColorTexture;
    vec3 diffuseTxt = texture(textureSamplers[nonuniformEXT(txtId)], fragTexCoord).xyz;
    diffuse *= diffuseTxt;
  }

  // Specular
  vec3 specular = computeSpecular(mat, viewDir, L, N);

  // Result
  outColor = vec4(lightIntensity * (diffuse + specular), 1);
>>>>>>> b7ba7d4fdc4128cefb7ab1d6097cc6d21c37d817
}
