/* Copyright (c) 2014-2018, NVIDIA CORPORATION. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of NVIDIA CORPORATION nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <sstream>
#include <vulkan/vulkan.hpp>

extern std::vector<std::string> defaultSearchPaths;

<<<<<<< HEAD
#define STB_IMAGE_IMPLEMENTATION
#include <random>

#include "fileformats/stb_image.h"
#include "fileformats/stb_image_write.h"
#include "hello_vulkan.h"
#include "nvvk/commands_vk.hpp"
#include "nvvk/pipeline_vk.hpp"
#include "obj_loader.h"
#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtx/transform.hpp>
#include <nvh/gltfscene.hpp>
#include <tiny_gltf.h>

#include "../parser/syntax.tab.hpp"
#include "../parser/tokenizer.h"
#include "io.h"
#include "logging.h"
#include "vk_utils.h"
#include "pbrt_scene.h"

// Holding the camera matrices
struct CameraMatrices {
    nvmath::mat4f view;
    nvmath::mat4f proj;
    nvmath::mat4f viewInverse;
    // #VKRay
    nvmath::mat4f projInverse;
=======
#define TINYGLTF_IMPLEMENTATION
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION


#include "hello_vulkan.h"
#include "nvh/cameramanipulator.hpp"
#include "nvh/fileoperations.hpp"
#include "nvh/gltfscene.hpp"
#include "nvh/nvprint.hpp"
#include "nvvk/commands_vk.hpp"
#include "nvvk/descriptorsets_vk.hpp"
#include "nvvk/pipeline_vk.hpp"
#include "nvvk/renderpasses_vk.hpp"
#include "nvvk/shaders_vk.hpp"

#include "nvh/alignment.hpp"
#include "shaders/binding.glsl"

// Holding the camera matrices
struct CameraMatrices
{
  nvmath::mat4f view;
  nvmath::mat4f proj;
  nvmath::mat4f viewInverse;
  // #VKRay
  nvmath::mat4f projInverse;
>>>>>>> b7ba7d4fdc4128cefb7ab1d6097cc6d21c37d817
};

//--------------------------------------------------------------------------------------------------
// Keep the handle on the device
// Initialize the tool to do all our allocations: buffers, images
//
<<<<<<< HEAD
void HelloVulkan::Setup(const vk::Instance& instance, const vk::Device& device,
                        const vk::PhysicalDevice& physicalDevice, u32 queueFamily)
{
    AppBase::Setup(instance, device, physicalDevice, queueFamily);
    allocator_.Setup(device, physicalDevice);
    CHECK(allocator_.ReadyToUse());
    m_debug.setup(device_);
=======
void HelloVulkan::setup(const vk::Instance&       instance,
                        const vk::Device&         device,
                        const vk::PhysicalDevice& physicalDevice,
                        uint32_t                  queueFamily)
{
  AppBase::setup(instance, device, physicalDevice, queueFamily);
  m_alloc.init(device, physicalDevice);
  m_debug.setup(m_device);
>>>>>>> b7ba7d4fdc4128cefb7ab1d6097cc6d21c37d817
}

//--------------------------------------------------------------------------------------------------
// Called at each frame to update the camera matrix
//
<<<<<<< HEAD
void HelloVulkan::UpdateUniformBuffer()
{
    // CameraManip.updateAnim();
    camera_navigator_->UpdateAnimation();
    const float aspectRatio = window_size_.width / static_cast<float>(window_size_.height);

    struct {
        glm::mat4 view;
        glm::mat4 proj;
        glm::mat4 view_inverse;
        glm::mat4 proj_inverse;
    } glm_ubo;
    glm_ubo.view = camera_navigator_->ViewMatrix();
    glm_ubo.proj =
        glm::perspective(glm::radians(camera_navigator_->Fov()), aspectRatio, 0.1f, 1000.0f);
    glm_ubo.proj[1][1] *= -1;
    glm_ubo.view_inverse = glm::inverse(glm_ubo.view);
    glm_ubo.proj_inverse = glm::inverse(glm_ubo.proj);

    void* data = device_.mapMemory(m_cameraMat.memory, 0, sizeof(glm_ubo));
    memcpy(data, &glm_ubo, sizeof(glm_ubo));
    device_.unmapMemory(m_cameraMat.memory);
=======
void HelloVulkan::updateUniformBuffer(const vk::CommandBuffer& cmdBuf)
{
  const float aspectRatio = m_size.width / static_cast<float>(m_size.height);

  CameraMatrices ubo = {};
  ubo.view           = CameraManip.getMatrix();
  ubo.proj           = nvmath::perspectiveVK(CameraManip.getFov(), aspectRatio, 0.1f, 1000.0f);
  // ubo.proj[1][1] *= -1;  // Inverting Y for Vulkan
  ubo.viewInverse = nvmath::invert(ubo.view);
  // #VKRay
  ubo.projInverse = nvmath::invert(ubo.proj);


  cmdBuf.updateBuffer<CameraMatrices>(m_cameraMat.buffer, 0, ubo);

  // Making sure the matrix buffer will be available
  vk::MemoryBarrier mb{vk::AccessFlagBits::eTransferWrite, vk::AccessFlagBits::eShaderRead};
  cmdBuf.pipelineBarrier(vk::PipelineStageFlagBits::eTransfer,
                         vk::PipelineStageFlagBits::eVertexShader
                             | vk::PipelineStageFlagBits::eAccelerationStructureBuildKHR,
                         vk::DependencyFlagBits::eDeviceGroup, {mb}, {}, {});
>>>>>>> b7ba7d4fdc4128cefb7ab1d6097cc6d21c37d817
}

//--------------------------------------------------------------------------------------------------
// Describing the layout pushed when rendering
//
<<<<<<< HEAD
void HelloVulkan::BuildDescriptorSetLayout()
{
    using vkDS     = vk::DescriptorSetLayoutBinding;
    using vkDT     = vk::DescriptorType;
    using vkSS     = vk::ShaderStageFlagBits;
    uint32_t nbTxt = static_cast<uint32_t>(m_textures.size());
    uint32_t nbObj = static_cast<uint32_t>(m_objModel.size());

    auto& bind = DS_layout_bindings_;
    bind.AddBinding(
        vkDS(kDsbCameraMatrices, vkDT::eUniformBuffer, 1, vkSS::eVertex | vkSS::eRaygenKHR));
    bind.AddBinding(
        vkDS(kDsbVertices, vkDT::eStorageBuffer, 1, vkSS::eClosestHitKHR | vkSS::eAnyHitKHR));
    bind.AddBinding(
        vkDS(kDsbIndices, vkDT::eStorageBuffer, 1, vkSS::eClosestHitKHR | vkSS::eAnyHitKHR));
    bind.AddBinding(vkDS(kDsbNormals, vkDT::eStorageBuffer, 1, vkSS::eClosestHitKHR));
    bind.AddBinding(vkDS(kDsbTexcoords, vkDT::eStorageBuffer, 1, vkSS::eClosestHitKHR));
    bind.AddBinding(vkDS(kDsbMaterials, vkDT::eStorageBuffer, 1,
                         vkSS::eFragment | vkSS::eClosestHitKHR | vkSS::eAnyHitKHR));
    bind.AddBinding(vkDS(kDsbMatrices, vkDT::eStorageBuffer, 1,
                         vkSS::eVertex | vkSS::eClosestHitKHR | vkSS::eAnyHitKHR));
    bind.AddBinding(vkDS(kDsbTextures, vkDT::eCombinedImageSampler, cast_u32(m_textures.size()),
                         vkSS::eFragment | vkSS::eClosestHitKHR | vkSS::eAnyHitKHR));

    m_descSetLayout = DS_layout_bindings_.MakeLayout(device_);
    m_descPool      = DS_layout_bindings_.MakePool(device_, 1);
    auto desc_sets  = device_.allocateDescriptorSets({m_descPool, 1, &m_descSetLayout});
    CHECK_EQ(desc_sets.size(), 1);
    m_descSet = desc_sets.front();
}

void HelloVulkan::LoadPbrtModel(const std::string& filename)
{
    scene_t       pbr_scene_ast;
    std::ifstream file_in(filename);
    if (!file_in) {
        LOG(ERROR) << "Can't open file " << filename << " for pbr input";
        return;
    }

    pbr::Tokenizer tokenizer(&file_in);
    tokenizer.set_directory(filename);
    pbr::Parser parser(tokenizer, &pbr_scene_ast);
    parser.parse();

    print_scene(pbr_scene_ast);
    pbr_scene.LoadFromAst(&pbr_scene_ast, tokenizer.file_root_path());
    
    return;
=======
void HelloVulkan::createDescriptorSetLayout()
{
  using vkDS     = vk::DescriptorSetLayoutBinding;
  using vkDT     = vk::DescriptorType;
  using vkSS     = vk::ShaderStageFlagBits;
  uint32_t nbTxt = static_cast<uint32_t>(m_textures.size());

  auto& bind = m_descSetLayoutBind;
  // Camera matrices (binding = 0)
  bind.addBinding(vkDS(B_CAMERA, vkDT::eUniformBuffer, 1, vkSS::eVertex | vkSS::eRaygenKHR));
  bind.addBinding(
      vkDS(B_VERTICES, vkDT::eStorageBuffer, 1, vkSS::eClosestHitKHR | vkSS::eAnyHitKHR));
  bind.addBinding(
      vkDS(B_INDICES, vkDT::eStorageBuffer, 1, vkSS::eClosestHitKHR | vkSS::eAnyHitKHR));
  bind.addBinding(vkDS(B_NORMALS, vkDT::eStorageBuffer, 1, vkSS::eClosestHitKHR));
  bind.addBinding(vkDS(B_TEXCOORDS, vkDT::eStorageBuffer, 1, vkSS::eClosestHitKHR));
  bind.addBinding(vkDS(B_MATERIALS, vkDT::eStorageBuffer, 1,
                       vkSS::eFragment | vkSS::eClosestHitKHR | vkSS::eAnyHitKHR));
  bind.addBinding(vkDS(B_MATRICES, vkDT::eStorageBuffer, 1,
                       vkSS::eVertex | vkSS::eClosestHitKHR | vkSS::eAnyHitKHR));
  auto nbTextures = static_cast<uint32_t>(m_textures.size());
  bind.addBinding(vkDS(B_TEXTURES, vkDT::eCombinedImageSampler, nbTextures,
                       vkSS::eFragment | vkSS::eClosestHitKHR | vkSS::eAnyHitKHR));


  m_descSetLayout = m_descSetLayoutBind.createLayout(m_device);
  m_descPool      = m_descSetLayoutBind.createPool(m_device, 1);
  m_descSet       = nvvk::allocateDescriptorSet(m_device, m_descPool, m_descSetLayout);
>>>>>>> b7ba7d4fdc4128cefb7ab1d6097cc6d21c37d817
}

//--------------------------------------------------------------------------------------------------
// Setting up the buffers in the descriptor set
//
<<<<<<< HEAD
void HelloVulkan::UpdateDescriptorSet()
{
    std::vector<vk::WriteDescriptorSet> writes;

    // Camera matrices and scene description
    vk::DescriptorBufferInfo dbiUnif       = {m_cameraMat.handle, 0, VK_WHOLE_SIZE};
    vk::DescriptorBufferInfo vertex_desc   = {scene_data_.vertex_buffer.handle, 0, VK_WHOLE_SIZE};
    vk::DescriptorBufferInfo index_desc    = {scene_data_.index_buffer.handle, 0, VK_WHOLE_SIZE};
    vk::DescriptorBufferInfo normal_desc   = {scene_data_.normal_buffer.handle, 0, VK_WHOLE_SIZE};
    vk::DescriptorBufferInfo uv_desc       = {scene_data_.uv_buffer.handle, 0, VK_WHOLE_SIZE};
    vk::DescriptorBufferInfo material_desc = {scene_data_.mtl_buffer.handle, 0, VK_WHOLE_SIZE};
    vk::DescriptorBufferInfo matrix_desc   = {scene_data_.matrix_buffer.handle, 0, VK_WHOLE_SIZE};

    writes.emplace_back(DS_layout_bindings_.MakeWrite(m_descSet, kDsbCameraMatrices, &dbiUnif));
    writes.emplace_back(DS_layout_bindings_.MakeWrite(m_descSet, kDsbVertices, &vertex_desc));
    writes.emplace_back(DS_layout_bindings_.MakeWrite(m_descSet, kDsbIndices, &index_desc));
    writes.emplace_back(DS_layout_bindings_.MakeWrite(m_descSet, kDsbNormals, &normal_desc));
    writes.emplace_back(DS_layout_bindings_.MakeWrite(m_descSet, kDsbTexcoords, &uv_desc));
    writes.emplace_back(DS_layout_bindings_.MakeWrite(m_descSet, kDsbMaterials, &material_desc));
    writes.emplace_back(DS_layout_bindings_.MakeWrite(m_descSet, kDsbMatrices, &matrix_desc));

    // All texture samplers
    std::vector<vk::DescriptorImageInfo> diit;
    for (auto& texture : m_textures) {
        diit.push_back(texture.descriptor);
    }
    writes.emplace_back(DS_layout_bindings_.MakeWriteArray(m_descSet, kDsbTextures, diit.data()));

    // Writing the information
    device_.updateDescriptorSets(static_cast<uint32_t>(writes.size()), writes.data(), 0, nullptr);
=======
void HelloVulkan::updateDescriptorSet()
{
  std::vector<vk::WriteDescriptorSet> writes;

  // Camera matrices and scene description
  vk::DescriptorBufferInfo dbiUnif{m_cameraMat.buffer, 0, VK_WHOLE_SIZE};
  vk::DescriptorBufferInfo vertexDesc{m_vertexBuffer.buffer, 0, VK_WHOLE_SIZE};
  vk::DescriptorBufferInfo indexDesc{m_indexBuffer.buffer, 0, VK_WHOLE_SIZE};
  vk::DescriptorBufferInfo normalDesc{m_normalBuffer.buffer, 0, VK_WHOLE_SIZE};
  vk::DescriptorBufferInfo uvDesc{m_uvBuffer.buffer, 0, VK_WHOLE_SIZE};
  vk::DescriptorBufferInfo materialDesc{m_materialBuffer.buffer, 0, VK_WHOLE_SIZE};
  vk::DescriptorBufferInfo matrixDesc{m_matrixBuffer.buffer, 0, VK_WHOLE_SIZE};

  writes.emplace_back(m_descSetLayoutBind.makeWrite(m_descSet, B_CAMERA, &dbiUnif));
  writes.emplace_back(m_descSetLayoutBind.makeWrite(m_descSet, B_VERTICES, &vertexDesc));
  writes.emplace_back(m_descSetLayoutBind.makeWrite(m_descSet, B_INDICES, &indexDesc));
  writes.emplace_back(m_descSetLayoutBind.makeWrite(m_descSet, B_NORMALS, &normalDesc));
  writes.emplace_back(m_descSetLayoutBind.makeWrite(m_descSet, B_TEXCOORDS, &uvDesc));
  writes.emplace_back(m_descSetLayoutBind.makeWrite(m_descSet, B_MATERIALS, &materialDesc));
  writes.emplace_back(m_descSetLayoutBind.makeWrite(m_descSet, B_MATRICES, &matrixDesc));

  // All texture samplers
  std::vector<vk::DescriptorImageInfo> diit;
  for(auto& texture : m_textures)
    diit.emplace_back(texture.descriptor);
  writes.emplace_back(m_descSetLayoutBind.makeWriteArray(m_descSet, B_TEXTURES, diit.data()));

  // Writing the information
  m_device.updateDescriptorSets(static_cast<uint32_t>(writes.size()), writes.data(), 0, nullptr);
>>>>>>> b7ba7d4fdc4128cefb7ab1d6097cc6d21c37d817
}

//--------------------------------------------------------------------------------------------------
// Creating the pipeline layout
//
<<<<<<< HEAD
void HelloVulkan::BuildGraphicsPipeline()
{
    using vkSS = vk::ShaderStageFlagBits;

    vk::PushConstantRange pushConstantRanges = {vkSS::eVertex | vkSS::eFragment, 0,
                                                sizeof(ObjPushConstant)};

    // Creating the Pipeline Layout
    vk::PipelineLayoutCreateInfo pipelineLayoutCreateInfo;
    vk::DescriptorSetLayout      descSetLayout(m_descSetLayout);
    pipelineLayoutCreateInfo.setSetLayoutCount(1);
    pipelineLayoutCreateInfo.setPSetLayouts(&descSetLayout);
    pipelineLayoutCreateInfo.setPushConstantRangeCount(1);
    pipelineLayoutCreateInfo.setPPushConstantRanges(&pushConstantRanges);
    m_pipelineLayout = device_.createPipelineLayout(pipelineLayoutCreateInfo);

    // Creating the Pipeline
    std::vector<std::string>                paths = defaultSearchPaths;
    nvvk::GraphicsPipelineGeneratorCombined gpb(device_, m_pipelineLayout, m_offscreenRenderPass);
    gpb.depthStencilState.depthTestEnable = true;
    gpb.addShader(io::LoadBinaryFile("shaders/vert_shader.vert.spv", paths), vkSS::eVertex);
    gpb.addShader(io::LoadBinaryFile("shaders/frag_shader.frag.spv", paths), vkSS::eFragment);

    // Describes how vertex data should be interpreted (as attributes).
    gpb.addBindingDescriptions(
        {{0, sizeof(glm::vec3)}, {1, sizeof(glm::vec3)}, {2, sizeof(glm::vec2)}});
    gpb.addAttributeDescriptions(
        // loc, binding, format, offset
        {{0, 0, vk::Format::eR32G32B32Sfloat, 0},
         {1, 1, vk::Format::eR32G32B32Sfloat, 0},
         {2, 2, vk::Format::eR32G32Sfloat, 0}});

    m_graphicsPipeline = gpb.createPipeline();
    m_debug.setObjectName(m_graphicsPipeline, "Graphics");
=======
void HelloVulkan::createGraphicsPipeline()
{
  using vkSS = vk::ShaderStageFlagBits;

  vk::PushConstantRange pushConstantRanges = {vkSS::eVertex | vkSS::eFragment, 0,
                                              sizeof(ObjPushConstant)};

  // Creating the Pipeline Layout
  vk::PipelineLayoutCreateInfo pipelineLayoutCreateInfo;
  vk::DescriptorSetLayout      descSetLayout(m_descSetLayout);
  pipelineLayoutCreateInfo.setSetLayoutCount(1);
  pipelineLayoutCreateInfo.setPSetLayouts(&descSetLayout);
  pipelineLayoutCreateInfo.setPushConstantRangeCount(1);
  pipelineLayoutCreateInfo.setPPushConstantRanges(&pushConstantRanges);
  m_pipelineLayout = m_device.createPipelineLayout(pipelineLayoutCreateInfo);

  // Creating the Pipeline
  std::vector<std::string>                paths = defaultSearchPaths;
  nvvk::GraphicsPipelineGeneratorCombined gpb(m_device, m_pipelineLayout, m_offscreenRenderPass);
  gpb.depthStencilState.depthTestEnable = true;
  gpb.addShader(nvh::loadFile("shaders/vert_shader.vert.spv", true, paths, true), vkSS::eVertex);
  gpb.addShader(nvh::loadFile("shaders/frag_shader.frag.spv", true, paths, true), vkSS::eFragment);
  gpb.addBindingDescriptions(
      {{0, sizeof(nvmath::vec3)}, {1, sizeof(nvmath::vec3)}, {2, sizeof(nvmath::vec2)}});
  gpb.addAttributeDescriptions({
      {0, 0, vk::Format::eR32G32B32Sfloat, 0},  // Position
      {1, 1, vk::Format::eR32G32B32Sfloat, 0},  // Normal
      {2, 2, vk::Format::eR32G32Sfloat, 0},     // Texcoord0
  });
  m_graphicsPipeline = gpb.createPipeline();
  m_debug.setObjectName(m_graphicsPipeline, "Graphics");
>>>>>>> b7ba7d4fdc4128cefb7ab1d6097cc6d21c37d817
}

//--------------------------------------------------------------------------------------------------
// Loading the OBJ file and setting up all buffers
//
<<<<<<< HEAD
void HelloVulkan::LoadModel(const std::string& filename, glm::mat4 transform)
{
    using vkBU = vk::BufferUsageFlagBits;

    ObjLoader loader;
    loader.loadModel(filename);

    // Converting from Srgb to linear
    for (auto& m : loader.m_materials) {
        m.ambient  = nvmath::pow(m.ambient, 2.2f);
        m.diffuse  = nvmath::pow(m.diffuse, 2.2f);
        m.specular = nvmath::pow(m.specular, 2.2f);
    }

    ObjInstance instance;
    instance.objIndex    = static_cast<uint32_t>(m_objModel.size());
    instance.transform   = transform;
    instance.transformIT = glm::transpose(glm::inverse(transform));
    instance.txtOffset   = static_cast<uint32_t>(m_textures.size());

    ObjModel model;
    model.nbIndices  = static_cast<uint32_t>(loader.m_indices.size());
    model.nbVertices = static_cast<uint32_t>(loader.m_vertices.size());

    // Create the buffers on Device and copy vertices, indices and materials
    vkpbr::CommandPool cmd_pool(device_, graphics_queue_index_);
    vk::CommandBuffer  cmdBuf = cmd_pool.MakeCmdBuffer();

    model.vertexBuffer   = allocator_.MakeBuffer(cmdBuf, loader.m_vertices,
                                               vkBU::eVertexBuffer | vkBU::eStorageBuffer
                                                   | vkBU::eShaderDeviceAddress);
    model.indexBuffer    = allocator_.MakeBuffer(cmdBuf, loader.m_indices,
                                              vkBU::eIndexBuffer | vkBU::eStorageBuffer
                                                  | vkBU::eShaderDeviceAddress);
    model.matColorBuffer = allocator_.MakeBuffer(cmdBuf, loader.m_materials, vkBU::eStorageBuffer);
    model.matIndexBuffer = allocator_.MakeBuffer(cmdBuf, loader.m_matIndx, vkBU::eStorageBuffer);
    // Creates all textures found
    BuildTextureImages(cmdBuf, loader.m_textures);
    cmd_pool.SubmitAndWait(cmdBuf);
    allocator_.ReleaseAllStagingBuffers();

    std::string objNb = std::to_string(instance.objIndex);
    m_debug.setObjectName(model.vertexBuffer.handle, (std::string("vertex_" + objNb).c_str()));
    m_debug.setObjectName(model.indexBuffer.handle, (std::string("index_" + objNb).c_str()));
    m_debug.setObjectName(model.matColorBuffer.handle, (std::string("mat_" + objNb).c_str()));
    m_debug.setObjectName(model.matIndexBuffer.handle, (std::string("matIdx_" + objNb).c_str()));

    m_objModel.emplace_back(model);
    m_objInstance.emplace_back(instance);
}

std::string MaterialSummary(const nvh::GltfMaterial& mtl)
{
    auto ColorName = [](nvmath::vec4 color) {
        float       brightness = std::max(std::max(color[0], color[1]), color[2]);
        std::string brightness_str;
        if (brightness < 0.01) {
            brightness_str = "black";
            return brightness_str;
        } else if (brightness < 0.33) {
            brightness_str = "dark";
        } else if (brightness < 0.67) {
            brightness_str = "";
        } else {
            brightness_str = "bright";
        }
        color /= brightness;
        int code = 0;
        code |= (color[0] > 0.5 ? 1 : 0);
        code |= (color[1] > 0.5 ? 2 : 0);
        code |= (color[2] > 0.5 ? 4 : 0);
        const char* kHues[] = {"gray", "red",     "green", "yellow",
                               "blue", "magenta", "cyan",  "white"};
        std::string hue     = "gray";
        if (brightness > 0.9999 && code == 7) {
            hue            = "white";
            brightness_str = "";
        } else if (code != 0) {
            hue = kHues[code];
        }
        return brightness_str + " " + hue;
    };

    std::ostringstream description;
    if (mtl.shadingModel == 0) {
        // metallic-roughness
        description << "pbrBaseColor = " << (mtl.pbrBaseColorTexture >= 0 ? "Textured " : "Solid ")
                    << ColorName(mtl.pbrBaseColorFactor) << ", ";
        description << (mtl.pbrMetallicFactor < 0.5 ? "highly" : "slightly") << " metallic, ";
        description << (mtl.pbrRoughnessFactor < 0.5 ? "highly" : "slightly") << " rough, ";
    } else {
        description << "diffuse = " << (mtl.khrDiffuseTexture >= 0 ? "Textured" : "Solid ")
                    << ColorName(mtl.khrDiffuseFactor) << ", ";
        description << "specular/glossy = "
                    << (mtl.khrSpecularGlossinessTexture >= 0 ? "Textured " : "Solid")
                    << ColorName(mtl.khrSpecularFactor);
    }

    // Emissivity
    description << "Emission: " << (mtl.emissiveTexture >= 0 ? "Textured " : "Solid ")
                << ColorName(mtl.emissiveFactor) << ", ";
    if (mtl.alphaMode != 0)
        description << "alpha = " << mtl.alphaCutoff << ", ";
    if (mtl.doubleSided)
        description << "double sided, ";
    if (mtl.normalTexture >= 0)
        description << "normal textured with scale = " << mtl.normalTextureScale << ", ";
    if (mtl.occlusionTexture >= 0)
        description << "occlusion textured with scale = " << mtl.occlusionTextureStrength;
    return description.str();
}

void HelloVulkan::LoadGltfModel(const std::string& filename, glm::mat4 transform)
{
    using vkBU = vk::BufferUsageFlagBits;
    // Loads the glTF Scene.
    // --------------------------------------------------------------------------------------------
    tinygltf::Model    t_model;
    tinygltf::TinyGLTF t_context;
    std::string        warning, error;
    if (!t_context.LoadASCIIFromFile(&t_model, &error, &warning, filename)) {
        LOG(FATAL) << "Error while loading scene";
    }
    gltf_scene_.importMaterials(t_model);
    gltf_scene_.importDrawableNodes(t_model,
                                    nvh::GltfAttributes::Normal | nvh::GltfAttributes::Texcoord_0);

    LOG(INFO) << gltf_scene_.m_materials.size() << " material(s):";
    for (const auto& mtl : gltf_scene_.m_materials) {
        std::cout << " - " << MaterialSummary(mtl) << '\n';
    }
    putchar('\n');

    vkpbr::CommandPool cmd_pool(device_, graphics_queue_index_);
    auto               cmd_buffer = cmd_pool.MakeCmdBuffer();

    scene_data_.vertex_buffer = allocator_.MakeBuffer(cmd_buffer, gltf_scene_.m_positions,
                                                      vkBU::eVertexBuffer | vkBU::eStorageBuffer
                                                          | vkBU::eShaderDeviceAddress);
    scene_data_.index_buffer  = allocator_.MakeBuffer(cmd_buffer, gltf_scene_.m_indices,
                                                     vkBU::eIndexBuffer | vkBU::eStorageBuffer
                                                         | vkBU::eShaderDeviceAddress);
    scene_data_.normal_buffer = allocator_.MakeBuffer(cmd_buffer, gltf_scene_.m_normals,
                                                      vkBU::eVertexBuffer | vkBU::eStorageBuffer);

    scene_data_.uv_buffer = allocator_.MakeBuffer(cmd_buffer, gltf_scene_.m_texcoords0,
                                                  vkBU::eVertexBuffer | vkBU::eStorageBuffer);
    scene_data_.mtl_buffer =
        allocator_.MakeBuffer(cmd_buffer, gltf_scene_.m_materials, vkBU::eStorageBuffer);

    std::vector<nvmath::mat4f> node_matrices;
    for (const auto& node : gltf_scene_.m_nodes) {
        node_matrices.emplace_back(node.worldMatrix.mat_array);
    }
    scene_data_.matrix_buffer =
        allocator_.MakeBuffer(cmd_buffer, node_matrices, vkBU::eStorageBuffer);

    // Prepares data for finding primitive mesh information in the closest-hit shader.
    std::vector<RtPrimitiveLookup> prim_lookup;
    for (auto& mesh : gltf_scene_.m_primMeshes) {
        prim_lookup.push_back({mesh.firstIndex, mesh.vertexOffset, mesh.materialIndex});
    }
    scene_data_.rt_prim_lookup_buffer =
        allocator_.MakeBuffer(cmd_buffer, prim_lookup, vkBU::eStorageBuffer);

    // Creates all textures in the scene.
    BuildTextureImages(cmd_buffer, t_model);
    cmd_pool.SubmitAndWait(cmd_buffer);
    allocator_.ReleaseAllStagingBuffers();
}
=======
void HelloVulkan::loadScene(const std::string& filename)
{
  using vkBU = vk::BufferUsageFlagBits;
  tinygltf::Model    tmodel;
  tinygltf::TinyGLTF tcontext;
  std::string        warn, error;

  LOGI("Loading file: %s", filename.c_str());
  if(!tcontext.LoadASCIIFromFile(&tmodel, &error, &warn, filename))
  {
    assert(!"Error while loading scene");
  }
  LOGW(warn.c_str());
  LOGE(error.c_str());


  m_gltfScene.importMaterials(tmodel);
  m_gltfScene.importDrawableNodes(tmodel,
                                  nvh::GltfAttributes::Normal | nvh::GltfAttributes::Texcoord_0);

  // Create the buffers on Device and copy vertices, indices and materials
  nvvk::CommandPool cmdBufGet(m_device, m_graphicsQueueIndex);
  vk::CommandBuffer cmdBuf = cmdBufGet.createCommandBuffer();

  m_vertexBuffer =
      m_alloc.createBuffer(cmdBuf, m_gltfScene.m_positions,
                           vkBU::eVertexBuffer | vkBU::eStorageBuffer | vkBU::eShaderDeviceAddress
                               | vkBU::eAccelerationStructureBuildInputReadOnlyKHR);
  m_indexBuffer =
      m_alloc.createBuffer(cmdBuf, m_gltfScene.m_indices,
                           vkBU::eIndexBuffer | vkBU::eStorageBuffer | vkBU::eShaderDeviceAddress
                               | vkBU::eAccelerationStructureBuildInputReadOnlyKHR);
  m_normalBuffer   = m_alloc.createBuffer(cmdBuf, m_gltfScene.m_normals,
                                        vkBU::eVertexBuffer | vkBU::eStorageBuffer);
  m_uvBuffer       = m_alloc.createBuffer(cmdBuf, m_gltfScene.m_texcoords0,
                                    vkBU::eVertexBuffer | vkBU::eStorageBuffer);
  m_materialBuffer = m_alloc.createBuffer(cmdBuf, m_gltfScene.m_materials, vkBU::eStorageBuffer);

  // Instance Matrices used by rasterizer
  std::vector<nvmath::mat4f> nodeMatrices;
  for(auto& node : m_gltfScene.m_nodes)
  {
    nodeMatrices.emplace_back(node.worldMatrix);
  }
  m_matrixBuffer = m_alloc.createBuffer(cmdBuf, nodeMatrices, vkBU::eStorageBuffer);

  // The following is used to find the primitive mesh information in the CHIT
  std::vector<RtPrimitiveLookup> primLookup;
  for(auto& primMesh : m_gltfScene.m_primMeshes)
  {
    primLookup.push_back({primMesh.firstIndex, primMesh.vertexOffset, primMesh.materialIndex});
  }
  m_rtPrimLookup =
      m_alloc.createBuffer(cmdBuf, primLookup, vk::BufferUsageFlagBits::eStorageBuffer);


  // Creates all textures found
  createTextureImages(cmdBuf, tmodel);
  cmdBufGet.submitAndWait(cmdBuf);
  m_alloc.finalizeAndReleaseStaging();

  m_debug.setObjectName(m_vertexBuffer.buffer, "Vertex");
  m_debug.setObjectName(m_indexBuffer.buffer, "Index");
  m_debug.setObjectName(m_normalBuffer.buffer, "Normal");
  m_debug.setObjectName(m_uvBuffer.buffer, "TexCoord");
  m_debug.setObjectName(m_materialBuffer.buffer, "Material");
  m_debug.setObjectName(m_matrixBuffer.buffer, "Matrix");
}

>>>>>>> b7ba7d4fdc4128cefb7ab1d6097cc6d21c37d817

//--------------------------------------------------------------------------------------------------
// Creating the uniform buffer holding the camera matrices
// - Buffer is host visible
//
<<<<<<< HEAD
void HelloVulkan::BuildUniformBuffer()
{
    using vkBU = vk::BufferUsageFlagBits;
    using vkMP = vk::MemoryPropertyFlagBits;

    m_cameraMat = allocator_.MakeBuffer(sizeof(CameraMatrices), vkBU::eUniformBuffer,
                                        vkMP::eHostVisible | vkMP::eHostCoherent);
    m_debug.setObjectName(m_cameraMat.handle, "cameraMat");
}

//--------------------------------------------------------------------------------------------------
// Create a storage buffer containing the description of the scene elements
// - Which geometry is used by which instance
// - Transformation
// - Offset for texture
//
void HelloVulkan::BuildSceneDescriptionBuffer()
{
    using vkBU = vk::BufferUsageFlagBits;
    vkpbr::CommandPool cmd_pool(device_, graphics_queue_index_);
    auto               cmd_buffer = cmd_pool.MakeCmdBuffer();
    m_sceneDesc = allocator_.MakeBuffer(cmd_buffer, m_objInstance, vkBU::eStorageBuffer);
    cmd_pool.SubmitAndWait(cmd_buffer);
    allocator_.ReleaseAllStagingBuffers();

    m_debug.setObjectName(m_sceneDesc.handle, "sceneDesc");
=======
void HelloVulkan::createUniformBuffer()
{
  using vkBU = vk::BufferUsageFlagBits;
  using vkMP = vk::MemoryPropertyFlagBits;

  m_cameraMat = m_alloc.createBuffer(sizeof(CameraMatrices),
                                     vkBU::eUniformBuffer | vkBU::eTransferDst, vkMP::eDeviceLocal);
  m_debug.setObjectName(m_cameraMat.buffer, "cameraMat");
>>>>>>> b7ba7d4fdc4128cefb7ab1d6097cc6d21c37d817
}

//--------------------------------------------------------------------------------------------------
// Creating all textures and samplers
//
<<<<<<< HEAD
void HelloVulkan::BuildTextureImages(const vk::CommandBuffer&        cmdBuf,
                                     const std::vector<std::string>& textures)
{
    using vkIU = vk::ImageUsageFlagBits;

    vk::SamplerCreateInfo samplerCreateInfo{
        {}, vk::Filter::eLinear, vk::Filter::eLinear, vk::SamplerMipmapMode::eLinear};
    samplerCreateInfo.setMaxLod(FLT_MAX);
    vk::Format format = vk::Format::eR8G8B8A8Srgb;

    // If no textures are present, create a dummy one to accommodate the pipeline layout
    if (textures.empty() && m_textures.empty()) {
        vkpbr::UniqueMemoryTexture texture;

        std::array<uint8_t, 4> color{255u, 255u, 255u, 255u};
        vk::DeviceSize         bufferSize      = sizeof(color);
        auto                   imgSize         = vk::Extent2D(1, 1);
        auto                   imageCreateInfo = vkpbr::MakeImage2DCreateInfo(imgSize, format);

        // Creating the dummy texure
        vkpbr::UniqueMemoryImage image =
            allocator_.MakeImage(cmdBuf, bufferSize, color.data(), imageCreateInfo);

        vk::ImageViewCreateInfo ivInfo =
            vkpbr::MakeImageViewCreateInfo(image.handle, imageCreateInfo);
        texture = allocator_.MakeTexture(image, ivInfo, samplerCreateInfo);
        CHECK(texture.descriptor.sampler);

        // The image format must be in VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL
        vkpbr::CmdBarrierImageLayout(cmdBuf, texture.handle, vk::ImageLayout::eUndefined,
                                     vk::ImageLayout::eShaderReadOnlyOptimal,
                                     vk::ImageAspectFlagBits::eColor);
        m_textures.push_back(texture);
    } else {
        // Uploading all images
        for (const auto& texture : textures) {
            std::stringstream o;
            int               texWidth, texHeight, texChannels;
            o << "media/textures/" << texture;
            std::string txtFile = io::FindFile(o.str(), defaultSearchPaths);

            stbi_uc* pixels =
                stbi_load(txtFile.c_str(), &texWidth, &texHeight, &texChannels, STBI_rgb_alpha);

            // Handle failure
            if (!pixels) {
                texWidth = texHeight = 1;
                texChannels          = 4;
                std::array<uint8_t, 4> color{255u, 0u, 255u, 255u};
                pixels = reinterpret_cast<stbi_uc*>(color.data());
            }

            vk::DeviceSize bufferSize =
                static_cast<uint64_t>(texWidth) * texHeight * sizeof(uint8_t) * 4;
            auto imgSize         = vk::Extent2D(texWidth, texHeight);
            auto imageCreateInfo = vkpbr::MakeImage2DCreateInfo(imgSize, format, vkIU::eSampled)
                                       .setMipLevels(vkpbr::MipLevels(imgSize));

            {
                vkpbr::UniqueMemoryImage image =
                    allocator_.MakeImage(cmdBuf, bufferSize, pixels, imageCreateInfo);
                vkpbr::CmdGenerateMipmaps(cmdBuf, image.handle, format, imgSize,
                                          imageCreateInfo.mipLevels);
                auto image_view_ci = vk::ImageViewCreateInfo()
                                         .setImage(image.handle)
                                         .setFormat(imageCreateInfo.format)
                                         .setViewType(vk::ImageViewType::e2D);
                image_view_ci.subresourceRange.setAspectMask(vk::ImageAspectFlagBits::eColor)
                    .setBaseMipLevel(0)
                    .setLevelCount(VK_REMAINING_MIP_LEVELS)
                    .setBaseArrayLayer(0)
                    .setLayerCount(VK_REMAINING_ARRAY_LAYERS);
                vkpbr::UniqueMemoryTexture texture =
                    allocator_.MakeTexture(image, image_view_ci, samplerCreateInfo);
                m_textures.push_back(texture);
            }
        }
    }
=======
void HelloVulkan::createTextureImages(const vk::CommandBuffer& cmdBuf, tinygltf::Model& gltfModel)
{
  using vkIU = vk::ImageUsageFlagBits;

  vk::SamplerCreateInfo samplerCreateInfo{
      {}, vk::Filter::eLinear, vk::Filter::eLinear, vk::SamplerMipmapMode::eLinear};
  samplerCreateInfo.setMaxLod(FLT_MAX);
  vk::Format format = vk::Format::eR8G8B8A8Srgb;

  auto addDefaultTexture = [this]() {
    // Make dummy image(1,1), needed as we cannot have an empty array
    nvvk::ScopeCommandBuffer cmdBuf(m_device, m_graphicsQueueIndex);
    std::array<uint8_t, 4>   white = {255, 255, 255, 255};
    m_textures.emplace_back(m_alloc.createTexture(
        cmdBuf, 4, white.data(), nvvk::makeImage2DCreateInfo(vk::Extent2D{1, 1}), {}));
    m_debug.setObjectName(m_textures.back().image, "dummy");
  };

  if(gltfModel.images.empty())
  {
    addDefaultTexture();
    return;
  }

  m_textures.reserve(gltfModel.images.size());
  for(size_t i = 0; i < gltfModel.images.size(); i++)
  {
    auto&        gltfimage  = gltfModel.images[i];
    void*        buffer     = &gltfimage.image[0];
    VkDeviceSize bufferSize = gltfimage.image.size();
    auto         imgSize    = vk::Extent2D(gltfimage.width, gltfimage.height);

    if(bufferSize == 0 || gltfimage.width == -1 || gltfimage.height == -1)
    {
      addDefaultTexture();
      continue;
    }

    vk::ImageCreateInfo imageCreateInfo =
        nvvk::makeImage2DCreateInfo(imgSize, format, vkIU::eSampled, true);

    nvvk::Image image = m_alloc.createImage(cmdBuf, bufferSize, buffer, imageCreateInfo);
    nvvk::cmdGenerateMipmaps(cmdBuf, image.image, format, imgSize, imageCreateInfo.mipLevels);
    vk::ImageViewCreateInfo ivInfo = nvvk::makeImageViewCreateInfo(image.image, imageCreateInfo);
    m_textures.emplace_back(m_alloc.createTexture(image, ivInfo, samplerCreateInfo));

    m_debug.setObjectName(m_textures[i].image, std::string("Txt" + std::to_string(i)).c_str());
  }
>>>>>>> b7ba7d4fdc4128cefb7ab1d6097cc6d21c37d817
}

//--------------------------------------------------------------------------------------------------
// Destroying all allocations
//
void HelloVulkan::destroyResources()
{
<<<<<<< HEAD
    device_.destroy(m_graphicsPipeline);
    device_.destroy(m_pipelineLayout);
    device_.destroy(m_descPool);
    device_.destroy(m_descSetLayout);
    m_cameraMat.DestroyFrom(device_);
    m_sceneDesc.DestroyFrom(device_);

    for (auto& m : m_objModel) {
        m.vertexBuffer.DestroyFrom(device_);
        m.indexBuffer.DestroyFrom(device_);
        m.matColorBuffer.DestroyFrom(device_);
        m.matIndexBuffer.DestroyFrom(device_);
    }

    for (auto& t : m_textures) {
        t.DestroyFrom(device_);
    }

    // Destroys resource from the scene data.
    scene_data_.vertex_buffer.DestroyFrom(device_);
    scene_data_.index_buffer.DestroyFrom(device_);
    scene_data_.normal_buffer.DestroyFrom(device_);
    scene_data_.uv_buffer.DestroyFrom(device_);
    scene_data_.mtl_buffer.DestroyFrom(device_);
    scene_data_.matrix_buffer.DestroyFrom(device_);
    scene_data_.rt_prim_lookup_buffer.DestroyFrom(device_);

    //#Post
    device_.destroy(m_postPipeline);
    device_.destroy(m_postPipelineLayout);
    device_.destroy(m_postDescPool);
    device_.destroy(m_postDescSetLayout);
    m_offscreenColor.DestroyFrom(device_);
    m_offscreenDepth.DestroyFrom(device_);
    device_.destroy(m_offscreenRenderPass);
    device_.destroy(m_offscreenFramebuffer);

    // #VKRay
    m_rtBuilder.destroy();
    device_.destroy(m_rtDescPool);
    device_.destroy(m_rtDescSetLayout);
    device_.destroy(m_rtPipeline);
    device_.destroy(m_rtPipelineLayout);
    m_rtSBTBuffer.DestroyFrom(device_);

    m_spheresBuffer.DestroyFrom(device_);
    m_spheresAabbBuffer.DestroyFrom(device_);
    m_spheresMatColorBuffer.DestroyFrom(device_);
    m_spheresMatIndexBuffer.DestroyFrom(device_);
=======
  m_device.destroy(m_graphicsPipeline);
  m_device.destroy(m_pipelineLayout);
  m_device.destroy(m_descPool);
  m_device.destroy(m_descSetLayout);
  m_alloc.destroy(m_cameraMat);

  m_alloc.destroy(m_vertexBuffer);
  m_alloc.destroy(m_normalBuffer);
  m_alloc.destroy(m_uvBuffer);
  m_alloc.destroy(m_indexBuffer);
  m_alloc.destroy(m_materialBuffer);
  m_alloc.destroy(m_matrixBuffer);
  m_alloc.destroy(m_rtPrimLookup);

  for(auto& t : m_textures)
  {
    m_alloc.destroy(t);
  }

  //#Post
  m_device.destroy(m_postPipeline);
  m_device.destroy(m_postPipelineLayout);
  m_device.destroy(m_postDescPool);
  m_device.destroy(m_postDescSetLayout);
  m_alloc.destroy(m_offscreenColor);
  m_alloc.destroy(m_offscreenDepth);
  m_device.destroy(m_offscreenRenderPass);
  m_device.destroy(m_offscreenFramebuffer);

  // #VKRay
  m_rtBuilder.destroy();
  m_device.destroy(m_rtDescPool);
  m_device.destroy(m_rtDescSetLayout);
  m_device.destroy(m_rtPipeline);
  m_device.destroy(m_rtPipelineLayout);
  m_alloc.destroy(m_rtSBTBuffer);
>>>>>>> b7ba7d4fdc4128cefb7ab1d6097cc6d21c37d817
}

//--------------------------------------------------------------------------------------------------
// Drawing the scene in raster mode
//
void HelloVulkan::rasterize(const vk::CommandBuffer& cmdBuf)
{
<<<<<<< HEAD
    using vkPBP = vk::PipelineBindPoint;
    using vkSS  = vk::ShaderStageFlagBits;
    vk::DeviceSize offset{0};

    m_debug.beginLabel(cmdBuf, "Rasterize");

    // Dynamic Viewport
    cmdBuf.setViewport(
        0, {vk::Viewport(0, 0, (float)window_size_.width, (float)window_size_.height, 0, 1)});
    cmdBuf.setScissor(0, {{{0, 0}, {window_size_.width, window_size_.height}}});

    // Drawing all triangles
    cmdBuf.bindPipeline(vkPBP::eGraphics, m_graphicsPipeline);
    cmdBuf.bindDescriptorSets(vkPBP::eGraphics, m_pipelineLayout, 0, {m_descSet}, {});
    if constexpr (false) {
        for (int i = 0; i < m_objInstance.size(); ++i) {
            auto& inst                = m_objInstance[i];
            auto& model               = m_objModel[inst.objIndex];
            m_pushConstant.instanceId = i;  // Telling which instance is drawn
            cmdBuf.pushConstants<ObjPushConstant>(m_pipelineLayout, vkSS::eVertex | vkSS::eFragment,
                                                  0, m_pushConstant);

            cmdBuf.bindVertexBuffers(0, {model.vertexBuffer.handle}, {offset});
            cmdBuf.bindIndexBuffer(model.indexBuffer.handle, 0, vk::IndexType::eUint32);
            cmdBuf.drawIndexed(model.nbIndices, 1, 0, 0, 0);
        }
    } else {
        std::vector<vk::Buffer> vertex_buffers = {scene_data_.vertex_buffer.handle,
                                                  scene_data_.normal_buffer.handle,
                                                  scene_data_.uv_buffer.handle};
        cmdBuf.bindVertexBuffers(/*firstBinding =*/0, vertex_buffers, /*offsets=*/{0, 0, 0});
        cmdBuf.bindIndexBuffer(scene_data_.index_buffer.handle, /*offset=*/0,
                               vk::IndexType::eUint32);
        for (size_t node_i = 0; node_i < gltf_scene_.m_nodes.size(); ++node_i) {
            const auto& node      = gltf_scene_.m_nodes[node_i];
            const auto& primitive = gltf_scene_.m_primMeshes[node.primMesh];

            m_pushConstant.instanceId = node_i;
            m_pushConstant.materialId = primitive.materialIndex;
            cmdBuf.pushConstants<ObjPushConstant>(m_pipelineLayout, vkSS::eVertex | vkSS::eFragment,
                                                  /*offset=*/0, m_pushConstant);
            cmdBuf.drawIndexed(primitive.indexCount, /*instanceCount=*/1, primitive.firstIndex,
                               primitive.vertexOffset, /*firstInstance=*/0);
        }
    }
    m_debug.endLabel(cmdBuf);
=======
  using vkPBP = vk::PipelineBindPoint;
  using vkSS  = vk::ShaderStageFlagBits;

  std::vector<vk::DeviceSize> offsets = {0, 0, 0};

  m_debug.beginLabel(cmdBuf, "Rasterize");

  // Dynamic Viewport
  cmdBuf.setViewport(0, {vk::Viewport(0, 0, (float)m_size.width, (float)m_size.height, 0, 1)});
  cmdBuf.setScissor(0, {{{0, 0}, {m_size.width, m_size.height}}});

  // Drawing all triangles
  cmdBuf.bindPipeline(vkPBP::eGraphics, m_graphicsPipeline);
  cmdBuf.bindDescriptorSets(vkPBP::eGraphics, m_pipelineLayout, 0, {m_descSet}, {});
  std::vector<vk::Buffer> vertexBuffers = {m_vertexBuffer.buffer, m_normalBuffer.buffer,
                                           m_uvBuffer.buffer};
  cmdBuf.bindVertexBuffers(0, static_cast<uint32_t>(vertexBuffers.size()), vertexBuffers.data(),
                           offsets.data());
  cmdBuf.bindIndexBuffer(m_indexBuffer.buffer, 0, vk::IndexType::eUint32);

  uint32_t idxNode = 0;
  for(auto& node : m_gltfScene.m_nodes)
  {
    auto& primitive = m_gltfScene.m_primMeshes[node.primMesh];

    m_pushConstant.instanceId = idxNode++;
    m_pushConstant.materialId = primitive.materialIndex;
    cmdBuf.pushConstants<ObjPushConstant>(
        m_pipelineLayout, vk::ShaderStageFlagBits::eVertex | vk::ShaderStageFlagBits::eFragment, 0,
        m_pushConstant);
    cmdBuf.drawIndexed(primitive.indexCount, 1, primitive.firstIndex, primitive.vertexOffset, 0);
  }

  m_debug.endLabel(cmdBuf);
>>>>>>> b7ba7d4fdc4128cefb7ab1d6097cc6d21c37d817
}

//--------------------------------------------------------------------------------------------------
// Handling resize of the window
//
<<<<<<< HEAD
void HelloVulkan::WindowResizeCallback(int w, int h)
{
    AppBase::WindowResizeCallback(w, h);
    createOffscreenRender();
    updatePostDescriptorSet();
    updateRtDescriptorSet();
=======
void HelloVulkan::onResize(int /*w*/, int /*h*/)
{
  createOffscreenRender();
  updatePostDescriptorSet();
  updateRtDescriptorSet();
  resetFrame();
>>>>>>> b7ba7d4fdc4128cefb7ab1d6097cc6d21c37d817
}

//////////////////////////////////////////////////////////////////////////
// Post-processing
//////////////////////////////////////////////////////////////////////////

//--------------------------------------------------------------------------------------------------
// Creating an offscreen frame buffer and the associated render pass
//
void HelloVulkan::createOffscreenRender()
{
<<<<<<< HEAD
    // Creating the color image
    {
        auto colorCreateInfo = vkpbr::MakeImage2DCreateInfo(window_size_, m_offscreenColorFormat,
                                                            vk::ImageUsageFlagBits::eColorAttachment
                                                                | vk::ImageUsageFlagBits::eSampled
                                                                | vk::ImageUsageFlagBits::eStorage);

        vkpbr::UniqueMemoryImage image = allocator_.MakeImage(colorCreateInfo);
        vk::ImageViewCreateInfo  ivInfo =
            vkpbr::MakeImageViewCreateInfo(image.handle, colorCreateInfo);
        m_offscreenColor = allocator_.MakeTexture(image, ivInfo, vk::SamplerCreateInfo());
        m_offscreenColor.descriptor.imageLayout = vk::ImageLayout::eGeneral;
    }

    // Creating the depth buffer
    auto depthCreateInfo =
        vkpbr::MakeImage2DCreateInfo(window_size_, m_offscreenDepthFormat,
                                     vk::ImageUsageFlagBits::eDepthStencilAttachment);
    {
        vkpbr::UniqueMemoryImage image = allocator_.MakeImage(depthCreateInfo);

        vk::ImageViewCreateInfo depthStencilView;
        depthStencilView.setViewType(vk::ImageViewType::e2D);
        depthStencilView.setFormat(m_offscreenDepthFormat);
        depthStencilView.setSubresourceRange({vk::ImageAspectFlagBits::eDepth, 0, 1, 0, 1});
        depthStencilView.setImage(image.handle);

        m_offscreenDepth = allocator_.MakeTexture(image, depthStencilView, vk::SamplerCreateInfo{});
    }

    // Setting the image layout for both color and depth
    {
        vkpbr::CommandPool genCmdBuf(device_, graphics_queue_index_);
        auto               cmdBuf = genCmdBuf.MakeCmdBuffer();
        vkpbr::CmdBarrierImageLayout(cmdBuf, m_offscreenColor.handle, vk::ImageLayout::eUndefined,
                                     vk::ImageLayout::eGeneral, vk::ImageAspectFlagBits::eColor);
        vkpbr::CmdBarrierImageLayout(cmdBuf, m_offscreenDepth.handle, vk::ImageLayout::eUndefined,
                                     vk::ImageLayout::eDepthStencilAttachmentOptimal,
                                     vk::ImageAspectFlagBits::eDepth);

        genCmdBuf.SubmitAndWait(cmdBuf);
    }

    // Creating a renderpass for the offscreen
    if (!m_offscreenRenderPass) {
        m_offscreenRenderPass =
            vkpbr::MakeRenderPass(device_, {m_offscreenColorFormat}, m_offscreenDepthFormat, 1,
                                  /*clear_color=*/true, /*clear_depth=*/true,
                                  vk::ImageLayout::eGeneral, vk::ImageLayout::eGeneral);
    }

    // Creating the frame buffer for offscreen
    std::vector<vk::ImageView> attachments = {m_offscreenColor.descriptor.imageView,
                                              m_offscreenDepth.descriptor.imageView};

    device_.destroy(m_offscreenFramebuffer);
    vk::FramebufferCreateInfo info;
    info.setRenderPass(m_offscreenRenderPass);
    info.setAttachmentCount(2);
    info.setPAttachments(attachments.data());
    info.setWidth(window_size_.width);
    info.setHeight(window_size_.height);
    info.setLayers(1);
    m_offscreenFramebuffer = device_.createFramebuffer(info);
=======
  m_alloc.destroy(m_offscreenColor);
  m_alloc.destroy(m_offscreenDepth);

  // Creating the color image
  {
    auto colorCreateInfo = nvvk::makeImage2DCreateInfo(m_size, m_offscreenColorFormat,
                                                       vk::ImageUsageFlagBits::eColorAttachment
                                                           | vk::ImageUsageFlagBits::eSampled
                                                           | vk::ImageUsageFlagBits::eStorage);


    nvvk::Image             image  = m_alloc.createImage(colorCreateInfo);
    vk::ImageViewCreateInfo ivInfo = nvvk::makeImageViewCreateInfo(image.image, colorCreateInfo);
    m_offscreenColor               = m_alloc.createTexture(image, ivInfo, vk::SamplerCreateInfo());
    m_offscreenColor.descriptor.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
  }

  // Creating the depth buffer
  auto depthCreateInfo =
      nvvk::makeImage2DCreateInfo(m_size, m_offscreenDepthFormat,
                                  vk::ImageUsageFlagBits::eDepthStencilAttachment);
  {
    nvvk::Image image = m_alloc.createImage(depthCreateInfo);

    vk::ImageViewCreateInfo depthStencilView;
    depthStencilView.setViewType(vk::ImageViewType::e2D);
    depthStencilView.setFormat(m_offscreenDepthFormat);
    depthStencilView.setSubresourceRange({vk::ImageAspectFlagBits::eDepth, 0, 1, 0, 1});
    depthStencilView.setImage(image.image);

    m_offscreenDepth = m_alloc.createTexture(image, depthStencilView);
  }

  // Setting the image layout for both color and depth
  {
    nvvk::CommandPool genCmdBuf(m_device, m_graphicsQueueIndex);
    auto              cmdBuf = genCmdBuf.createCommandBuffer();
    nvvk::cmdBarrierImageLayout(cmdBuf, m_offscreenColor.image, vk::ImageLayout::eUndefined,
                                vk::ImageLayout::eGeneral);
    nvvk::cmdBarrierImageLayout(cmdBuf, m_offscreenDepth.image, vk::ImageLayout::eUndefined,
                                vk::ImageLayout::eDepthStencilAttachmentOptimal,
                                vk::ImageAspectFlagBits::eDepth);

    genCmdBuf.submitAndWait(cmdBuf);
  }

  // Creating a renderpass for the offscreen
  if(!m_offscreenRenderPass)
  {
    m_offscreenRenderPass =
        nvvk::createRenderPass(m_device, {m_offscreenColorFormat}, m_offscreenDepthFormat, 1, true,
                               true, vk::ImageLayout::eGeneral, vk::ImageLayout::eGeneral);
  }

  // Creating the frame buffer for offscreen
  std::vector<vk::ImageView> attachments = {m_offscreenColor.descriptor.imageView,
                                            m_offscreenDepth.descriptor.imageView};

  m_device.destroy(m_offscreenFramebuffer);
  vk::FramebufferCreateInfo info;
  info.setRenderPass(m_offscreenRenderPass);
  info.setAttachmentCount(2);
  info.setPAttachments(attachments.data());
  info.setWidth(m_size.width);
  info.setHeight(m_size.height);
  info.setLayers(1);
  m_offscreenFramebuffer = m_device.createFramebuffer(info);
>>>>>>> b7ba7d4fdc4128cefb7ab1d6097cc6d21c37d817
}

//--------------------------------------------------------------------------------------------------
// The pipeline is how things are rendered, which shaders, type of primitives, depth test and more
//
void HelloVulkan::createPostPipeline()
{
<<<<<<< HEAD
    // Push constants in the fragment shader
    vk::PushConstantRange pushConstantRanges = {vk::ShaderStageFlagBits::eFragment, 0,
                                                sizeof(float)};

    // Creating the pipeline layout
    vk::PipelineLayoutCreateInfo pipelineLayoutCreateInfo;
    pipelineLayoutCreateInfo.setSetLayoutCount(1);
    pipelineLayoutCreateInfo.setPSetLayouts(&m_postDescSetLayout);
    pipelineLayoutCreateInfo.setPushConstantRangeCount(1);
    pipelineLayoutCreateInfo.setPPushConstantRanges(&pushConstantRanges);
    m_postPipelineLayout = device_.createPipelineLayout(pipelineLayoutCreateInfo);

    // Pipeline: completely generic, no vertices
    std::vector<std::string> paths = defaultSearchPaths;

    nvvk::GraphicsPipelineGeneratorCombined pipelineGenerator(device_, m_postPipelineLayout,
                                                              render_pass_);
    pipelineGenerator.addShader(io::LoadBinaryFile("shaders/passthrough.vert.spv", paths),
                                vk::ShaderStageFlagBits::eVertex);
    pipelineGenerator.addShader(io::LoadBinaryFile("shaders/post.frag.spv", paths),
                                vk::ShaderStageFlagBits::eFragment);
    pipelineGenerator.rasterizationState.setCullMode(vk::CullModeFlagBits::eNone);
    m_postPipeline = pipelineGenerator.createPipeline();
    m_debug.setObjectName(m_postPipeline, "post");
=======
  // Push constants in the fragment shader
  vk::PushConstantRange pushConstantRanges = {vk::ShaderStageFlagBits::eFragment, 0, sizeof(float)};

  // Creating the pipeline layout
  vk::PipelineLayoutCreateInfo pipelineLayoutCreateInfo;
  pipelineLayoutCreateInfo.setSetLayoutCount(1);
  pipelineLayoutCreateInfo.setPSetLayouts(&m_postDescSetLayout);
  pipelineLayoutCreateInfo.setPushConstantRangeCount(1);
  pipelineLayoutCreateInfo.setPPushConstantRanges(&pushConstantRanges);
  m_postPipelineLayout = m_device.createPipelineLayout(pipelineLayoutCreateInfo);

  // Pipeline: completely generic, no vertices
  std::vector<std::string> paths = defaultSearchPaths;

  nvvk::GraphicsPipelineGeneratorCombined pipelineGenerator(m_device, m_postPipelineLayout,
                                                            m_renderPass);
  pipelineGenerator.addShader(nvh::loadFile("shaders/passthrough.vert.spv", true, paths, true),
                              vk::ShaderStageFlagBits::eVertex);
  pipelineGenerator.addShader(nvh::loadFile("shaders/post.frag.spv", true, paths, true),
                              vk::ShaderStageFlagBits::eFragment);
  pipelineGenerator.rasterizationState.setCullMode(vk::CullModeFlagBits::eNone);
  m_postPipeline = pipelineGenerator.createPipeline();
  m_debug.setObjectName(m_postPipeline, "post");
>>>>>>> b7ba7d4fdc4128cefb7ab1d6097cc6d21c37d817
}

//--------------------------------------------------------------------------------------------------
// The descriptor layout is the description of the data that is passed to the vertex or the
// fragment program.
//
void HelloVulkan::createPostDescriptor()
{
<<<<<<< HEAD
    using vkDS = vk::DescriptorSetLayoutBinding;
    using vkDT = vk::DescriptorType;
    using vkSS = vk::ShaderStageFlagBits;

    post_DS_layout_bindings_.AddBinding(vkDS(0, vkDT::eCombinedImageSampler, 1, vkSS::eFragment));
    m_postDescSetLayout = post_DS_layout_bindings_.MakeLayout(device_);
    m_postDescPool      = post_DS_layout_bindings_.MakePool(device_);
    auto desc_sets      = device_.allocateDescriptorSets({m_postDescPool, 1, &m_postDescSetLayout});
    CHECK_EQ(desc_sets.size(), 1);
    m_postDescSet = desc_sets.front();
=======
  using vkDS = vk::DescriptorSetLayoutBinding;
  using vkDT = vk::DescriptorType;
  using vkSS = vk::ShaderStageFlagBits;

  m_postDescSetLayoutBind.addBinding(vkDS(0, vkDT::eCombinedImageSampler, 1, vkSS::eFragment));
  m_postDescSetLayout = m_postDescSetLayoutBind.createLayout(m_device);
  m_postDescPool      = m_postDescSetLayoutBind.createPool(m_device);
  m_postDescSet       = nvvk::allocateDescriptorSet(m_device, m_postDescPool, m_postDescSetLayout);
>>>>>>> b7ba7d4fdc4128cefb7ab1d6097cc6d21c37d817
}

//--------------------------------------------------------------------------------------------------
// Update the output
//
void HelloVulkan::updatePostDescriptorSet()
{
<<<<<<< HEAD
    vk::WriteDescriptorSet writeDescriptorSets = post_DS_layout_bindings_.MakeWrite(
        m_postDescSet, 0,
        reinterpret_cast<const vk::DescriptorImageInfo*>(&m_offscreenColor.descriptor));
    device_.updateDescriptorSets(writeDescriptorSets, nullptr);
=======
  vk::WriteDescriptorSet writeDescriptorSets =
      m_postDescSetLayoutBind.makeWrite(m_postDescSet, 0, &m_offscreenColor.descriptor);
  m_device.updateDescriptorSets(writeDescriptorSets, nullptr);
>>>>>>> b7ba7d4fdc4128cefb7ab1d6097cc6d21c37d817
}

//--------------------------------------------------------------------------------------------------
// Draw a full screen quad with the attached image
//
void HelloVulkan::drawPost(vk::CommandBuffer cmdBuf)
{
<<<<<<< HEAD
    m_debug.beginLabel(cmdBuf, "Post");

    cmdBuf.setViewport(
        0, {vk::Viewport(0, 0, (float)window_size_.width, (float)window_size_.height, 0, 1)});
    cmdBuf.setScissor(0, {{{0, 0}, {window_size_.width, window_size_.height}}});

    auto aspectRatio =
        static_cast<float>(window_size_.width) / static_cast<float>(window_size_.height);
    cmdBuf.pushConstants<float>(m_postPipelineLayout, vk::ShaderStageFlagBits::eFragment, 0,
                                aspectRatio);
    cmdBuf.bindPipeline(vk::PipelineBindPoint::eGraphics, m_postPipeline);
    cmdBuf.bindDescriptorSets(vk::PipelineBindPoint::eGraphics, m_postPipelineLayout, 0,
                              m_postDescSet, {});
    cmdBuf.draw(3, 1, 0, 0);

    m_debug.endLabel(cmdBuf);
=======
  m_debug.beginLabel(cmdBuf, "Post");

  cmdBuf.setViewport(0, {vk::Viewport(0, 0, (float)m_size.width, (float)m_size.height, 0, 1)});
  cmdBuf.setScissor(0, {{{0, 0}, {m_size.width, m_size.height}}});

  auto aspectRatio = static_cast<float>(m_size.width) / static_cast<float>(m_size.height);
  cmdBuf.pushConstants<float>(m_postPipelineLayout, vk::ShaderStageFlagBits::eFragment, 0,
                              aspectRatio);
  cmdBuf.bindPipeline(vk::PipelineBindPoint::eGraphics, m_postPipeline);
  cmdBuf.bindDescriptorSets(vk::PipelineBindPoint::eGraphics, m_postPipelineLayout, 0,
                            m_postDescSet, {});
  cmdBuf.draw(3, 1, 0, 0);

  m_debug.endLabel(cmdBuf);
>>>>>>> b7ba7d4fdc4128cefb7ab1d6097cc6d21c37d817
}

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

<<<<<<< HEAD
void HelloVulkan::WriteFramebuffer(const std::string& file_name)
{
    auto subresource_range = vk::ImageSubresourceRange()
                                 .setAspectMask(vk::ImageAspectFlagBits::eColor)
                                 .setBaseArrayLayer(0)
                                 .setBaseMipLevel(0)
                                 .setLayerCount(1)
                                 .setLevelCount(1);

    vkpbr::CommandPool cmd_pool(device_, graphics_queue_index_);
    auto               cmd_buffer = cmd_pool.MakeCmdBuffer();
    vkpbr::CmdBarrierImageLayout(cmd_buffer, m_offscreenColor.handle, vk::ImageLayout::eGeneral,
                                 vk::ImageLayout::eTransferSrcOptimal, subresource_range);

    vk::MemoryRequirements2          memory_requirements;
    vk::MemoryDedicatedRequirements  dedicated_requirements;
    vk::ImageMemoryRequirementsInfo2 image_requirements_info;

    image_requirements_info.image = m_offscreenColor.handle;
    memory_requirements.pNext     = &dedicated_requirements;
    device_.getImageMemoryRequirements2(&image_requirements_info, &memory_requirements);

    auto staging_buffer = allocator_.MakeBuffer(
        memory_requirements.memoryRequirements.size, vk::BufferUsageFlagBits::eTransferDst,
        vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent);

    auto buffer_copy_region = vk::BufferImageCopy().setImageExtent(vk::Extent3D{window_size_, 1});
    buffer_copy_region.imageSubresource.setAspectMask(vk::ImageAspectFlagBits::eColor)
        .setLayerCount(1);
    cmd_buffer.copyImageToBuffer(m_offscreenColor.handle, vk::ImageLayout::eTransferSrcOptimal,
                                 staging_buffer.handle, buffer_copy_region);

    vkpbr::CmdBarrierImageLayout(cmd_buffer, m_offscreenColor.handle,
                                 vk::ImageLayout::eTransferSrcOptimal, vk::ImageLayout::eGeneral,
                                 subresource_range);

    cmd_pool.SubmitAndWait(cmd_buffer);

    u64 num_pixels = cast_u64(window_size_.width) * window_size_.height;

    float* pixels = new float[num_pixels * 4];
    {
        void* staging_memory = device_.mapMemory(staging_buffer.memory, 0, VK_WHOLE_SIZE);
        memcpy(pixels, staging_memory, num_pixels * 4 * sizeof(float));
        device_.unmapMemory(staging_buffer.memory);
    }
    unsigned char* uchar_pixels = new unsigned char[num_pixels * 4];
    for (u64 i = 0; i < num_pixels * 4; ++i) {
        float pixel_f   = glm::clamp(pixels[i], 0.0f, 1.0f);
        pixel_f         = std::pow(pixel_f, 1.0f / 2.2f);
        uchar_pixels[i] = cast<u8>(pixel_f * 255);
    }

    stbi_write_bmp(file_name.c_str(), window_size_.width, window_size_.height, 4, uchar_pixels);

    staging_buffer.DestroyFrom(device_);
}

=======
>>>>>>> b7ba7d4fdc4128cefb7ab1d6097cc6d21c37d817
//--------------------------------------------------------------------------------------------------
// Initialize Vulkan ray tracing
// #VKRay
void HelloVulkan::initRayTracing()
{
<<<<<<< HEAD
    // Requesting ray tracing properties
    auto properties = gpu_.getProperties2<vk::PhysicalDeviceProperties2,
                                          vk::PhysicalDeviceRayTracingPropertiesKHR>();
    m_rtProperties  = properties.get<vk::PhysicalDeviceRayTracingPropertiesKHR>();
    m_rtBuilder.setup(device_, &allocator_, graphics_queue_index_);
}

//--------------------------------------------------------------------------------------------------
// Converting a OBJ primitive to the ray tracing geometry used for the BLAS
//
vkpbr::RaytracingBuilderKHR::Blas HelloVulkan::objectToVkGeometryKHR(const ObjModel& model)
{
    vk::AccelerationStructureCreateGeometryTypeInfoKHR asCreate;
    asCreate.setGeometryType(vk::GeometryTypeKHR::eTriangles);
    asCreate.setIndexType(vk::IndexType::eUint32);
    asCreate.setVertexFormat(vk::Format::eR32G32B32Sfloat);
    asCreate.setMaxPrimitiveCount(model.nbIndices / 3);  // Nb triangles
    asCreate.setMaxVertexCount(model.nbVertices);
    asCreate.setAllowsTransforms(VK_FALSE);  // No adding transformation matrices

    vk::DeviceAddress vertexAddress = device_.getBufferAddress({model.vertexBuffer.handle});
    vk::DeviceAddress indexAddress  = device_.getBufferAddress({model.indexBuffer.handle});
    vk::AccelerationStructureGeometryTrianglesDataKHR triangles;
    triangles.setVertexFormat(asCreate.vertexFormat);
    triangles.setVertexData(vertexAddress);
    triangles.setVertexStride(sizeof(VertexObj));
    triangles.setIndexType(asCreate.indexType);
    triangles.setIndexData(indexAddress);
    triangles.setTransformData({});

    // Setting up the build info of the acceleration
    vk::AccelerationStructureGeometryKHR asGeom;
    asGeom.setGeometryType(asCreate.geometryType);
    asGeom.setFlags(vk::GeometryFlagBitsKHR::eOpaque);
    asGeom.geometry.setTriangles(triangles);

    vk::AccelerationStructureBuildOffsetInfoKHR offset;
    offset.setFirstVertex(0);
    offset.setPrimitiveCount(asCreate.maxPrimitiveCount);
    offset.setPrimitiveOffset(0);
    offset.setTransformOffset(0);

    vkpbr::RaytracingBuilderKHR::Blas blas;
    blas.asGeometry.emplace_back(asGeom);
    blas.asCreateGeometryInfo.emplace_back(asCreate);
    blas.asBuildOffsetInfo.emplace_back(offset);
    return blas;
}

vkpbr::RaytracingBuilderKHR::Blas HelloVulkan::PrimitiveToGeometryKHR(const nvh::GltfPrimMesh& prim)
{
    auto as_create_geometry = vk::AccelerationStructureCreateGeometryTypeInfoKHR()
                                  .setGeometryType(vk::GeometryTypeKHR::eTriangles)
                                  .setIndexType(vk::IndexType::eUint32)
                                  .setVertexFormat(vk::Format::eR32G32B32Sfloat)
                                  .setMaxPrimitiveCount(prim.indexCount / 3)
                                  .setMaxVertexCount(prim.vertexCount)
                                  .setAllowsTransforms(VK_FALSE);  // No adding transforms.

    auto vertex_addr = device_.getBufferAddress({scene_data_.vertex_buffer.handle});
    auto index_addr  = device_.getBufferAddress({scene_data_.index_buffer.handle});

    auto triangles = vk::AccelerationStructureGeometryTrianglesDataKHR()
                         .setVertexFormat(as_create_geometry.vertexFormat)
                         .setVertexData(vertex_addr)
                         .setVertexStride(sizeof(nvmath::vec3f))
                         .setIndexType(as_create_geometry.indexType)
                         .setIndexData(index_addr);

    auto as_geometry = vk::AccelerationStructureGeometryKHR()
                           .setGeometryType(as_create_geometry.geometryType)
                           .setFlags(vk::GeometryFlagBitsKHR::eNoDuplicateAnyHitInvocation);
    as_geometry.geometry.setTriangles(triangles);

    auto offset = vk::AccelerationStructureBuildOffsetInfoKHR()
                      .setFirstVertex(prim.vertexOffset)
                      .setPrimitiveCount(prim.indexCount / 3)
                      .setPrimitiveOffset(prim.firstIndex * sizeof(u32))
                      .setTransformOffset(0);

    vkpbr::RaytracingBuilderKHR::Blas blas;
    blas.asGeometry.emplace_back(as_geometry);
    blas.asCreateGeometryInfo.emplace_back(as_create_geometry);
    blas.asBuildOffsetInfo.emplace_back(offset);
    return blas;
}

void HelloVulkan::BuildTextureImages(const vk::CommandBuffer& cmd_buffer,
                                     tinygltf::Model&         gltf_model)
{
    using vkIU = vk::ImageUsageFlagBits;

    auto sampler_ci = vk::SamplerCreateInfo({}, vk::Filter::eLinear, vk::Filter::eLinear,
                                            vk::SamplerMipmapMode::eLinear)
                          .setMaxLod(FLT_MAX);
    auto format = vk::Format::eR8G8B8A8Srgb;

    auto AddDefaultTexture = [cmd_buffer, this]() {
        // Makes dummy image(1, 1), needed as we cannot have an empty array.
        u8   white[4] = {255, 255, 255, 255};
        auto image =
            allocator_.MakeImage(cmd_buffer, 4, white, vkpbr::MakeImage2DCreateInfo({1, 1}));
        auto texture =
            allocator_.MakeTexture(image, vkpbr::MakeImage2DViewCreateInfo(image.handle), {});
        m_textures.emplace_back(std::move(texture));
    };

    if (gltf_model.images.empty()) {
        AddDefaultTexture();
        return;
    }

    m_textures.reserve(gltf_model.images.size());
    for (size_t i = 0; i < gltf_model.images.size(); ++i) {
        auto& gltf_image  = gltf_model.images[i];
        void* buffer      = &gltf_image.image[0];
        auto  buffer_size = gltf_image.image.size();
        auto  image_size  = vk::Extent2D(gltf_image.width, gltf_image.height);

        if (buffer_size == 0 || gltf_image.width <= 0 || gltf_image.height <= 0) {
            AddDefaultTexture();
            continue;
        }
        auto image_ci = vkpbr::MakeImage2DCreateInfo(image_size, format, vkIU::eSampled);
        vkpbr::UniqueMemoryImage image =
            allocator_.MakeImage(cmd_buffer, buffer_size, buffer, image_ci);
        vkpbr::CmdGenerateMipmaps(cmd_buffer, image.handle, format, image_size, image_ci.mipLevels);
        auto image_view_ci = vkpbr::MakeImageViewCreateInfo(image.handle, image_ci);
        m_textures.emplace_back(allocator_.MakeTexture(image, image_view_ci, sampler_ci));
    }
}

//--------------------------------------------------------------------------------------------------
// Returning the ray tracing geometry used for the BLAS, containing all spheres
//
vkpbr::RaytracingBuilderKHR::Blas HelloVulkan::sphereToVkGeometryKHR()
{
    vk::AccelerationStructureCreateGeometryTypeInfoKHR asCreate;
    asCreate.setGeometryType(vk::GeometryTypeKHR::eAabbs);
    asCreate.setMaxPrimitiveCount((uint32_t)m_spheres.size());  // Nb triangles
    asCreate.setIndexType(vk::IndexType::eNoneKHR);
    asCreate.setVertexFormat(vk::Format::eUndefined);
    asCreate.setMaxVertexCount(0);
    asCreate.setAllowsTransforms(VK_FALSE);  // No adding transformation matrices


    vk::DeviceAddress dataAddress = device_.getBufferAddress({m_spheresAabbBuffer.handle});
    vk::AccelerationStructureGeometryAabbsDataKHR aabbs;
    aabbs.setData(dataAddress);
    aabbs.setStride(sizeof(Aabb));

    // Setting up the build info of the acceleration
    vk::AccelerationStructureGeometryKHR asGeom;
    asGeom.setGeometryType(asCreate.geometryType);
    asGeom.setFlags(vk::GeometryFlagBitsKHR::eOpaque);
    asGeom.geometry.setAabbs(aabbs);

    vk::AccelerationStructureBuildOffsetInfoKHR offset;
    offset.setFirstVertex(0);
    offset.setPrimitiveCount(asCreate.maxPrimitiveCount);
    offset.setPrimitiveOffset(0);
    offset.setTransformOffset(0);

    vkpbr::RaytracingBuilderKHR::Blas blas;
    blas.asGeometry.emplace_back(asGeom);
    blas.asCreateGeometryInfo.emplace_back(asCreate);
    blas.asBuildOffsetInfo.emplace_back(offset);
    return blas;
}

//--------------------------------------------------------------------------------------------------
// Creating all spheres
//
void HelloVulkan::createSpheres()
{
    std::random_device                    rd{};
    std::mt19937                          gen{rd()};
    std::normal_distribution<float>       xzd{0.f, 5.f};
    std::normal_distribution<float>       yd{3.f, 1.f};
    std::uniform_real_distribution<float> radd{.05f, .2f};

    // All spheres
    Sphere s;
    for (uint32_t i = 0; i < 2000000; i++) {
        s.center = glm::vec3(xzd(gen), yd(gen), xzd(gen));
        s.radius = radd(gen);
        m_spheres.emplace_back(s);
    }

    // Axis aligned bounding box of each sphere
    std::vector<Aabb> aabbs;
    for (const auto& s : m_spheres) {
        Aabb aabb;
        aabb.minimum = s.center - glm::vec3(s.radius);
        aabb.maximum = s.center + glm::vec3(s.radius);
        aabbs.emplace_back(aabb);
    }

    // Creating two materials
    MaterialObj mat;
    mat.diffuse = nvmath::vec3f(0, 1, 1);
    std::vector<MaterialObj> materials;
    std::vector<int>         matIdx;
    materials.emplace_back(mat);
    mat.diffuse = nvmath::vec3f(1, 1, 0);
    materials.emplace_back(mat);

    // Assign a material to each sphere
    for (size_t i = 0; i < m_spheres.size(); i++) {
        matIdx.push_back(i % 2);
    }

    // Creating all buffers
    using vkBU = vk::BufferUsageFlagBits;

    vkpbr::CommandPool cmd_pool(device_, graphics_queue_index_);
    auto               cmdBuf = cmd_pool.MakeCmdBuffer();

    m_spheresBuffer         = allocator_.MakeBuffer(cmdBuf, m_spheres, vkBU::eStorageBuffer);
    m_spheresAabbBuffer     = allocator_.MakeBuffer(cmdBuf, aabbs, vkBU::eShaderDeviceAddress);
    m_spheresMatIndexBuffer = allocator_.MakeBuffer(cmdBuf, matIdx, vkBU::eStorageBuffer);
    m_spheresMatColorBuffer = allocator_.MakeBuffer(cmdBuf, materials, vkBU::eStorageBuffer);
    cmd_pool.SubmitAndWait(cmdBuf);

    // Debug information
    m_debug.setObjectName(m_spheresBuffer.handle, "spheres");
    m_debug.setObjectName(m_spheresAabbBuffer.handle, "spheresAabb");
    m_debug.setObjectName(m_spheresMatColorBuffer.handle, "spheresMat");
    m_debug.setObjectName(m_spheresMatIndexBuffer.handle, "spheresMatIdx");
}

void HelloVulkan::createBottomLevelAS()
{
    // BLAS - Storing each primitive in a geometry
    std::vector<vkpbr::RaytracingBuilderKHR::Blas> all_blas;

    // Builds the blas from the glTF scene data.
    all_blas.clear();
    all_blas.reserve(gltf_scene_.m_primMeshes.size());

    // all_blas.emplace_back(sphereToVkGeometryKHR());
    // all_blas.emplace_back(PrimitiveToGeometryKHR(gltf_scene_.m_primMeshes[0]));

    for (const auto& mesh : gltf_scene_.m_primMeshes) {
        all_blas.emplace_back(PrimitiveToGeometryKHR(mesh));
    }
    m_rtBuilder.buildBlas(all_blas, vk::BuildAccelerationStructureFlagBitsKHR::ePreferFastTrace);
=======
  // Requesting ray tracing properties
  auto properties =
      m_physicalDevice.getProperties2<vk::PhysicalDeviceProperties2,
                                      vk::PhysicalDeviceRayTracingPipelinePropertiesKHR>();
  m_rtProperties = properties.get<vk::PhysicalDeviceRayTracingPipelinePropertiesKHR>();
  m_rtBuilder.setup(m_device, &m_alloc, m_graphicsQueueIndex);
}

//--------------------------------------------------------------------------------------------------
// Converting a GLTF primitive in the Raytracing Geometry used for the BLAS
//
nvvk::RaytracingBuilderKHR::BlasInput HelloVulkan::primitiveToGeometry(
    const nvh::GltfPrimMesh& prim)
{
  // Building part
  vk::DeviceAddress vertexAddress = m_device.getBufferAddress({m_vertexBuffer.buffer});
  vk::DeviceAddress indexAddress  = m_device.getBufferAddress({m_indexBuffer.buffer});

  vk::AccelerationStructureGeometryTrianglesDataKHR triangles;
  triangles.setVertexFormat(vk::Format::eR32G32B32Sfloat);
  triangles.setVertexData(vertexAddress);
  triangles.setVertexStride(sizeof(nvmath::vec3f));
  triangles.setIndexType(vk::IndexType::eUint32);
  triangles.setIndexData(indexAddress);
  triangles.setTransformData({});
  triangles.setMaxVertex(prim.vertexCount);

  // Setting up the build info of the acceleration
  vk::AccelerationStructureGeometryKHR asGeom;
  asGeom.setGeometryType(vk::GeometryTypeKHR::eTriangles);
  asGeom.setFlags(vk::GeometryFlagBitsKHR::eNoDuplicateAnyHitInvocation);  // For AnyHit
  asGeom.geometry.setTriangles(triangles);

  vk::AccelerationStructureBuildRangeInfoKHR offset;
  offset.setFirstVertex(prim.vertexOffset);
  offset.setPrimitiveCount(prim.indexCount / 3);
  offset.setPrimitiveOffset(prim.firstIndex * sizeof(uint32_t));
  offset.setTransformOffset(0);

  nvvk::RaytracingBuilderKHR::BlasInput input;
  input.asGeometry.emplace_back(asGeom);
  input.asBuildOffsetInfo.emplace_back(offset);
  return input;
}

//--------------------------------------------------------------------------------------------------
//
//
void HelloVulkan::createBottomLevelAS()
{
  // BLAS - Storing each primitive in a geometry
  std::vector<nvvk::RaytracingBuilderKHR::BlasInput> allBlas;
  allBlas.reserve(m_gltfScene.m_primMeshes.size());
  for(auto& primMesh : m_gltfScene.m_primMeshes)
  {
    auto geo = primitiveToGeometry(primMesh);
    allBlas.push_back({geo});
  }
  m_rtBuilder.buildBlas(allBlas, vk::BuildAccelerationStructureFlagBitsKHR::ePreferFastTrace);
>>>>>>> b7ba7d4fdc4128cefb7ab1d6097cc6d21c37d817
}

void HelloVulkan::createTopLevelAS()
{
<<<<<<< HEAD

    std::vector<vkpbr::RaytracingBuilderKHR::Instance> tlas_instances;
    // Builds the TLAS for the gltf scene data.
    tlas_instances.clear();
    tlas_instances.reserve(gltf_scene_.m_nodes.size());
    for (const auto& node : gltf_scene_.m_nodes) {
        vkpbr::RaytracingBuilderKHR::Instance instance;
        memcpy(&instance.transform, &node.worldMatrix, sizeof instance.transform);
        instance.instanceId = node.primMesh;  // glInstanceCustomIndexEXT
        instance.blasId     = node.primMesh;
        instance.flags      = vk::GeometryInstanceFlagBitsKHR::eTriangleFacingCullDisable;
        instance.hitGroupId = 0;  // Uses the same hit group for all objects.
        tlas_instances.emplace_back(instance);
    }
    m_rtBuilder.buildTlas(tlas_instances,
                          vk::BuildAccelerationStructureFlagBitsKHR::ePreferFastTrace);
=======
  std::vector<nvvk::RaytracingBuilderKHR::Instance> tlas;
  tlas.reserve(m_gltfScene.m_nodes.size());
  uint32_t instID = 0;
  for(auto& node : m_gltfScene.m_nodes)
  {
    nvvk::RaytracingBuilderKHR::Instance rayInst;
    rayInst.transform  = node.worldMatrix;
    rayInst.instanceId = node.primMesh;  // gl_InstanceCustomIndexEXT: to find which primitive
    rayInst.blasId     = node.primMesh;
    rayInst.flags      = VK_GEOMETRY_INSTANCE_TRIANGLE_FACING_CULL_DISABLE_BIT_KHR;
    rayInst.hitGroupId = 0;  // We will use the same hit group for all objects
    tlas.emplace_back(rayInst);
  }
  m_rtBuilder.buildTlas(tlas, vk::BuildAccelerationStructureFlagBitsKHR::ePreferFastTrace);
>>>>>>> b7ba7d4fdc4128cefb7ab1d6097cc6d21c37d817
}

//--------------------------------------------------------------------------------------------------
// This descriptor set holds the Acceleration structure and the output image
//
void HelloVulkan::createRtDescriptorSet()
{
<<<<<<< HEAD
    using vkDT   = vk::DescriptorType;
    using vkSS   = vk::ShaderStageFlagBits;
    using vkDSLB = vk::DescriptorSetLayoutBinding;

    rt_DS_layout_bindings_.AddBinding(vkDSLB(kRtDsbAccelStruct, vkDT::eAccelerationStructureKHR, 1,
                                             vkSS::eRaygenKHR | vkSS::eClosestHitKHR));  // TLAS
    rt_DS_layout_bindings_.AddBinding(
        vkDSLB(kRtDsbOutputImage, vkDT::eStorageImage, 1, vkSS::eRaygenKHR));  // Output image
    rt_DS_layout_bindings_.AddBinding(
        vkDSLB(kRtDsbPrimInfo, vkDT::eStorageBuffer, 1, vkSS::eClosestHitKHR | vkSS::eAnyHitKHR));

    m_rtDescPool      = rt_DS_layout_bindings_.MakePool(device_);
    m_rtDescSetLayout = rt_DS_layout_bindings_.MakeLayout(device_);
    m_rtDescSet       = device_.allocateDescriptorSets({m_rtDescPool, 1, &m_rtDescSetLayout})[0];

    vk::AccelerationStructureKHR                   tlas = m_rtBuilder.getAccelerationStructure();
    vk::WriteDescriptorSetAccelerationStructureKHR descASInfo;
    descASInfo.setAccelerationStructureCount(1);
    descASInfo.setPAccelerationStructures(&tlas);
    vk::DescriptorImageInfo imageInfo{
        {}, m_offscreenColor.descriptor.imageView, vk::ImageLayout::eGeneral};
    vk::DescriptorBufferInfo prim_buffer_info{scene_data_.rt_prim_lookup_buffer.handle,
                                              /*offset =*/0, VK_WHOLE_SIZE};

    std::vector<vk::WriteDescriptorSet> writes;
    writes.emplace_back(rt_DS_layout_bindings_.MakeWrite(m_rtDescSet, 0, &descASInfo));
    writes.emplace_back(rt_DS_layout_bindings_.MakeWrite(m_rtDescSet, 1, &imageInfo));
    writes.emplace_back(
        rt_DS_layout_bindings_.MakeWrite(m_rtDescSet, kRtDsbPrimInfo, &prim_buffer_info));

    device_.updateDescriptorSets(static_cast<uint32_t>(writes.size()), writes.data(), 0, nullptr);
=======
  using vkDT   = vk::DescriptorType;
  using vkSS   = vk::ShaderStageFlagBits;
  using vkDSLB = vk::DescriptorSetLayoutBinding;

  m_rtDescSetLayoutBind.addBinding(vkDSLB(0, vkDT::eAccelerationStructureKHR, 1,
                                          vkSS::eRaygenKHR | vkSS::eClosestHitKHR));  // TLAS
  m_rtDescSetLayoutBind.addBinding(
      vkDSLB(1, vkDT::eStorageImage, 1, vkSS::eRaygenKHR));  // Output image
  m_rtDescSetLayoutBind.addBinding(vkDSLB(
      2, vkDT::eStorageBuffer, 1, vkSS::eClosestHitKHR | vkSS::eAnyHitKHR));  // Primitive info

  m_rtDescPool      = m_rtDescSetLayoutBind.createPool(m_device);
  m_rtDescSetLayout = m_rtDescSetLayoutBind.createLayout(m_device);
  m_rtDescSet       = m_device.allocateDescriptorSets({m_rtDescPool, 1, &m_rtDescSetLayout})[0];

  vk::AccelerationStructureKHR                   tlas = m_rtBuilder.getAccelerationStructure();
  vk::WriteDescriptorSetAccelerationStructureKHR descASInfo;
  descASInfo.setAccelerationStructureCount(1);
  descASInfo.setPAccelerationStructures(&tlas);
  vk::DescriptorImageInfo imageInfo{
      {}, m_offscreenColor.descriptor.imageView, vk::ImageLayout::eGeneral};
  vk::DescriptorBufferInfo primitiveInfoDesc{m_rtPrimLookup.buffer, 0, VK_WHOLE_SIZE};

  std::vector<vk::WriteDescriptorSet> writes;
  writes.emplace_back(m_rtDescSetLayoutBind.makeWrite(m_rtDescSet, 0, &descASInfo));
  writes.emplace_back(m_rtDescSetLayoutBind.makeWrite(m_rtDescSet, 1, &imageInfo));
  writes.emplace_back(m_rtDescSetLayoutBind.makeWrite(m_rtDescSet, 2, &primitiveInfoDesc));
  m_device.updateDescriptorSets(static_cast<uint32_t>(writes.size()), writes.data(), 0, nullptr);
>>>>>>> b7ba7d4fdc4128cefb7ab1d6097cc6d21c37d817
}


//--------------------------------------------------------------------------------------------------
// Writes the output image to the descriptor set
// - Required when changing resolution
//
void HelloVulkan::updateRtDescriptorSet()
{
<<<<<<< HEAD
    using vkDT = vk::DescriptorType;

    // (1) Output buffer
    vk::DescriptorImageInfo imageInfo{
        {}, m_offscreenColor.descriptor.imageView, vk::ImageLayout::eGeneral};
    vk::WriteDescriptorSet wds{m_rtDescSet, 1, 0, 1, vkDT::eStorageImage, &imageInfo};
    device_.updateDescriptorSets(wds, nullptr);
=======
  using vkDT = vk::DescriptorType;

  // (1) Output buffer
  vk::DescriptorImageInfo imageInfo{
      {}, m_offscreenColor.descriptor.imageView, vk::ImageLayout::eGeneral};
  vk::WriteDescriptorSet wds{m_rtDescSet, 1, 0, 1, vkDT::eStorageImage, &imageInfo};
  m_device.updateDescriptorSets(wds, nullptr);
>>>>>>> b7ba7d4fdc4128cefb7ab1d6097cc6d21c37d817
}


//--------------------------------------------------------------------------------------------------
// Pipeline for the ray tracer: all shaders, raygen, chit, miss
//
void HelloVulkan::createRtPipeline()
{
<<<<<<< HEAD
    std::vector<std::string> paths = defaultSearchPaths;

    vk::ShaderModule raygenSM =
        vkpbr::MakeShaderModule(device_, io::LoadBinaryFile("shaders/raytrace.rgen.spv", paths));
    vk::ShaderModule missSM =
        vkpbr::MakeShaderModule(device_, io::LoadBinaryFile("shaders/raytrace.rmiss.spv", paths));

    // The second miss shader is invoked when a shadow ray misses the geometry. It
    // simply indicates that no occlusion has been found
    vk::ShaderModule shadowmissSM =
        vkpbr::MakeShaderModule(device_,
                                io::LoadBinaryFile("shaders/raytraceShadow.rmiss.spv", paths));
    vk::ShaderModule chitSM =  // Hit Group0 - Closest Hit
        vkpbr::MakeShaderModule(device_, io::LoadBinaryFile("shaders/raytrace.rchit.spv", paths));
    // vk::ShaderModule chit2SM =  // Hit Group1 - Closest Hit + Intersection (procedural)
    // vkpbr::MakeShaderModule(device_, io::LoadBinaryFile("shaders/raytrace2.rchit.spv", paths));

    std::vector<vk::PipelineShaderStageCreateInfo> stages;

    auto MakeEmptyShaderGroupCI = []() {
        return vk::RayTracingShaderGroupCreateInfoKHR(vk::RayTracingShaderGroupTypeKHR::eGeneral,
                                                      VK_SHADER_UNUSED_KHR, VK_SHADER_UNUSED_KHR,
                                                      VK_SHADER_UNUSED_KHR, VK_SHADER_UNUSED_KHR);
    };

    stages.resize(kNumStages);
    stages[kRaygen]     = {{}, vk::ShaderStageFlagBits::eRaygenKHR, raygenSM, "main"};
    stages[kMiss]       = {{}, vk::ShaderStageFlagBits::eMissKHR, missSM, "main"};
    stages[kShadowMiss] = {{}, vk::ShaderStageFlagBits::eMissKHR, shadowmissSM, "main"};
    stages[kClosestHit] = {{}, vk::ShaderStageFlagBits::eClosestHitKHR, chitSM, "main"};
    // stages[kClosestHit2] = {{}, vk::ShaderStageFlagBits::eClosestHitKHR, chit2SM, "main"};

    // Raygen
    auto raygen_group = MakeEmptyShaderGroupCI().setGeneralShader(kRaygen);
    m_rtShaderGroups.push_back(raygen_group);
    // Miss
    auto miss_group = MakeEmptyShaderGroupCI().setGeneralShader(kMiss);
    m_rtShaderGroups.push_back(miss_group);
    // Shadow Miss
    miss_group.setGeneralShader(kShadowMiss);
    m_rtShaderGroups.push_back(miss_group);

    auto hit_group = MakeEmptyShaderGroupCI()
                         .setType(vk::RayTracingShaderGroupTypeKHR::eTrianglesHitGroup)
                         .setClosestHitShader(kClosestHit);
    m_rtShaderGroups.push_back(hit_group);
    // hit_group = MakeEmptyShaderGroupCI()
    //                .setType(vk::RayTracingShaderGroupTypeKHR::eProceduralHitGroup)
    //                .setClosestHitShader(kClosestHit2);
    // m_rtShaderGroups.push_back(hit_group);

    // Creates the RT pipeline layout: push constants and descriptor sets.
    // --------------------------------------------------------------------------------------------
    vk::PipelineLayoutCreateInfo pipelineLayoutCreateInfo;

    // Push constant: we want to be able to update constants used by the shaders
    vk::PushConstantRange pushConstant{vk::ShaderStageFlagBits::eRaygenKHR
                                           | vk::ShaderStageFlagBits::eClosestHitKHR
                                           | vk::ShaderStageFlagBits::eMissKHR,
                                       0, sizeof(RtPushConstant)};
    pipelineLayoutCreateInfo.setPushConstantRangeCount(1);
    pipelineLayoutCreateInfo.setPPushConstantRanges(&pushConstant);

    // Descriptor sets: one specific to ray tracing, and one shared with the rasterization pipeline
    std::vector<vk::DescriptorSetLayout> rtDescSetLayouts = {m_rtDescSetLayout, m_descSetLayout};
    pipelineLayoutCreateInfo.setSetLayoutCount(static_cast<uint32_t>(rtDescSetLayouts.size()));
    pipelineLayoutCreateInfo.setPSetLayouts(rtDescSetLayouts.data());

    m_rtPipelineLayout = device_.createPipelineLayout(pipelineLayoutCreateInfo);

    // Assemble the shader stages and recursion depth info into the ray tracing pipeline
    vk::RayTracingPipelineCreateInfoKHR rayPipelineInfo;
    rayPipelineInfo.setStageCount(static_cast<uint32_t>(stages.size()));  // Stages are shaders
    rayPipelineInfo.setPStages(stages.data());

    rayPipelineInfo.setGroupCount(cast_u32(m_rtShaderGroups.size()));  // 1-raygen, n-miss,
                                                                       // n-(hit[+anyhit+intersect])
    rayPipelineInfo.setPGroups(m_rtShaderGroups.data());

    rayPipelineInfo.setMaxRecursionDepth(2);  // Ray depth
    rayPipelineInfo.setLayout(m_rtPipelineLayout);
    m_rtPipeline = device_.createRayTracingPipelineKHR({}, rayPipelineInfo).value;

    for (auto& shader_module : {raygenSM, missSM, shadowmissSM, chitSM}) {
        device_.destroy(shader_module);
    }
=======
  std::vector<std::string> paths = defaultSearchPaths;

  vk::ShaderModule raygenSM =
      nvvk::createShaderModule(m_device,  //
                               nvh::loadFile("shaders/pathtrace.rgen.spv", true, paths, true));
  vk::ShaderModule missSM =
      nvvk::createShaderModule(m_device,  //
                               nvh::loadFile("shaders/pathtrace.rmiss.spv", true, paths, true));

  // The second miss shader is invoked when a shadow ray misses the geometry. It
  // simply indicates that no occlusion has been found
  vk::ShaderModule shadowmissSM = nvvk::createShaderModule(
      m_device, nvh::loadFile("shaders/raytraceShadow.rmiss.spv", true, paths, true));


  std::vector<vk::PipelineShaderStageCreateInfo> stages;

  // Raygen
  vk::RayTracingShaderGroupCreateInfoKHR rg{vk::RayTracingShaderGroupTypeKHR::eGeneral,
                                            VK_SHADER_UNUSED_KHR, VK_SHADER_UNUSED_KHR,
                                            VK_SHADER_UNUSED_KHR, VK_SHADER_UNUSED_KHR};
  stages.push_back({{}, vk::ShaderStageFlagBits::eRaygenKHR, raygenSM, "main"});
  rg.setGeneralShader(static_cast<uint32_t>(stages.size() - 1));
  m_rtShaderGroups.push_back(rg);
  // Miss
  vk::RayTracingShaderGroupCreateInfoKHR mg{vk::RayTracingShaderGroupTypeKHR::eGeneral,
                                            VK_SHADER_UNUSED_KHR, VK_SHADER_UNUSED_KHR,
                                            VK_SHADER_UNUSED_KHR, VK_SHADER_UNUSED_KHR};
  stages.push_back({{}, vk::ShaderStageFlagBits::eMissKHR, missSM, "main"});
  mg.setGeneralShader(static_cast<uint32_t>(stages.size() - 1));
  m_rtShaderGroups.push_back(mg);
  // Shadow Miss
  stages.push_back({{}, vk::ShaderStageFlagBits::eMissKHR, shadowmissSM, "main"});
  mg.setGeneralShader(static_cast<uint32_t>(stages.size() - 1));
  m_rtShaderGroups.push_back(mg);

  // Hit Group - Closest Hit + AnyHit
  vk::ShaderModule chitSM =
      nvvk::createShaderModule(m_device,  //
                               nvh::loadFile("shaders/pathtrace.rchit.spv", true, paths, true));

  vk::RayTracingShaderGroupCreateInfoKHR hg{vk::RayTracingShaderGroupTypeKHR::eTrianglesHitGroup,
                                            VK_SHADER_UNUSED_KHR, VK_SHADER_UNUSED_KHR,
                                            VK_SHADER_UNUSED_KHR, VK_SHADER_UNUSED_KHR};
  stages.push_back({{}, vk::ShaderStageFlagBits::eClosestHitKHR, chitSM, "main"});
  hg.setClosestHitShader(static_cast<uint32_t>(stages.size() - 1));
  m_rtShaderGroups.push_back(hg);

  vk::PipelineLayoutCreateInfo pipelineLayoutCreateInfo;

  // Push constant: we want to be able to update constants used by the shaders
  vk::PushConstantRange pushConstant{vk::ShaderStageFlagBits::eRaygenKHR
                                         | vk::ShaderStageFlagBits::eClosestHitKHR
                                         | vk::ShaderStageFlagBits::eMissKHR,
                                     0, sizeof(RtPushConstant)};
  pipelineLayoutCreateInfo.setPushConstantRangeCount(1);
  pipelineLayoutCreateInfo.setPPushConstantRanges(&pushConstant);

  // Descriptor sets: one specific to ray tracing, and one shared with the rasterization pipeline
  std::vector<vk::DescriptorSetLayout> rtDescSetLayouts = {m_rtDescSetLayout, m_descSetLayout};
  pipelineLayoutCreateInfo.setSetLayoutCount(static_cast<uint32_t>(rtDescSetLayouts.size()));
  pipelineLayoutCreateInfo.setPSetLayouts(rtDescSetLayouts.data());

  m_rtPipelineLayout = m_device.createPipelineLayout(pipelineLayoutCreateInfo);

  // Assemble the shader stages and recursion depth info into the ray tracing pipeline

  vk::RayTracingPipelineCreateInfoKHR rayPipelineInfo;
  rayPipelineInfo.setStageCount(static_cast<uint32_t>(stages.size()));  // Stages are shaders
  rayPipelineInfo.setPStages(stages.data());

  rayPipelineInfo.setGroupCount(static_cast<uint32_t>(
      m_rtShaderGroups.size()));  // 1-raygen, n-miss, n-(hit[+anyhit+intersect])
  rayPipelineInfo.setPGroups(m_rtShaderGroups.data());

  rayPipelineInfo.setMaxPipelineRayRecursionDepth(2);  // Ray depth
  rayPipelineInfo.setLayout(m_rtPipelineLayout);
  m_rtPipeline = static_cast<const vk::Pipeline&>(
      m_device.createRayTracingPipelineKHR({}, {}, rayPipelineInfo));

  m_device.destroy(raygenSM);
  m_device.destroy(missSM);
  m_device.destroy(shadowmissSM);
  m_device.destroy(chitSM);
>>>>>>> b7ba7d4fdc4128cefb7ab1d6097cc6d21c37d817
}

//--------------------------------------------------------------------------------------------------
// The Shader Binding Table (SBT)
// - getting all shader handles and writing them in a SBT buffer
// - Besides exception, this could be always done like this
//   See how the SBT buffer is used in run()
//
void HelloVulkan::createRtShaderBindingTable()
{
<<<<<<< HEAD
    auto groupCount      = cast_u32(m_rtShaderGroups.size());     // 3 shaders: raygen, miss chit
    u32  groupHandleSize = m_rtProperties.shaderGroupHandleSize;  // Size of a program identifier

    // Fetch all the shader handles used in the pipeline, so that they can be written in the SBT
    uint32_t sbtSize = groupCount * groupHandleSize;

    std::vector<uint8_t> shaderHandleStorage(sbtSize);
    device_.getRayTracingShaderGroupHandlesKHR(m_rtPipeline, 0, groupCount, sbtSize,
                                               shaderHandleStorage.data());
    // Write the handles in the SBT
    vkpbr::CommandPool cmd_pool(device_, graphics_queue_index_);
    vk::CommandBuffer  cmd_buffer = cmd_pool.MakeCmdBuffer();

    m_rtSBTBuffer = allocator_.MakeBuffer(cmd_buffer, shaderHandleStorage,
                                          vk::BufferUsageFlagBits::eRayTracingKHR);
    m_debug.setObjectName(m_rtSBTBuffer.handle, "SBT");

    cmd_pool.SubmitAndWait(cmd_buffer);
    allocator_.ReleaseAllStagingBuffers();
=======
  auto groupCount =
      static_cast<uint32_t>(m_rtShaderGroups.size());               // 3 shaders: raygen, miss, chit
  uint32_t groupHandleSize = m_rtProperties.shaderGroupHandleSize;  // Size of a program identifier
  uint32_t groupSizeAligned =
      nvh::align_up(groupHandleSize, m_rtProperties.shaderGroupBaseAlignment);

  // Fetch all the shader handles used in the pipeline, so that they can be written in the SBT
  uint32_t sbtSize = groupCount * groupSizeAligned;

  std::vector<uint8_t> shaderHandleStorage(sbtSize);
  auto result = m_device.getRayTracingShaderGroupHandlesKHR(m_rtPipeline, 0, groupCount, sbtSize,
                                                            shaderHandleStorage.data());
  if(result != vk::Result::eSuccess)
    LOGE("Fail getRayTracingShaderGroupHandlesKHR: %s", vk::to_string(result));

  // Write the handles in the SBT
  m_rtSBTBuffer = m_alloc.createBuffer(
      sbtSize,
      vk::BufferUsageFlagBits::eTransferSrc | vk::BufferUsageFlagBits::eShaderDeviceAddressKHR
          | vk::BufferUsageFlagBits::eShaderBindingTableKHR,
      vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent);
  m_debug.setObjectName(m_rtSBTBuffer.buffer, std::string("SBT").c_str());

  // Write the handles in the SBT
  void* mapped = m_alloc.map(m_rtSBTBuffer);
  auto* pData  = reinterpret_cast<uint8_t*>(mapped);
  for(uint32_t g = 0; g < groupCount; g++)
  {
    memcpy(pData, shaderHandleStorage.data() + g * groupHandleSize, groupHandleSize);  // raygen
    pData += groupSizeAligned;
  }
  m_alloc.unmap(m_rtSBTBuffer);


  m_alloc.finalizeAndReleaseStaging();
>>>>>>> b7ba7d4fdc4128cefb7ab1d6097cc6d21c37d817
}

//--------------------------------------------------------------------------------------------------
// Ray Tracing the scene
//
<<<<<<< HEAD
void HelloVulkan::raytrace(const vk::CommandBuffer& cmdBuf, const glm::vec4& clearColor)
{
    UpdateFrame();

    m_debug.beginLabel(cmdBuf, "Ray trace");
    // Initializing push constant values
    m_rtPushConstants.clearColor     = clearColor;
    m_rtPushConstants.lightPosition  = m_pushConstant.lightPosition;
    m_rtPushConstants.lightIntensity = m_pushConstant.lightIntensity;
    m_rtPushConstants.lightType      = m_pushConstant.lightType;

    cmdBuf.bindPipeline(vk::PipelineBindPoint::eRayTracingKHR, m_rtPipeline);
    cmdBuf.bindDescriptorSets(vk::PipelineBindPoint::eRayTracingKHR, m_rtPipelineLayout, 0,
                              {m_rtDescSet, m_descSet}, {});
    cmdBuf.pushConstants<RtPushConstant>(m_rtPipelineLayout,
                                         vk::ShaderStageFlagBits::eRaygenKHR
                                             | vk::ShaderStageFlagBits::eClosestHitKHR
                                             | vk::ShaderStageFlagBits::eMissKHR,
                                         0, m_rtPushConstants);
    // LOG(INFO) << "rtpc.length = " << m_rtPushConstants.path_length;

    vk::DeviceSize progSize = m_rtProperties.shaderGroupHandleSize;  // Size of a program identifier
    vk::DeviceSize rayGenOffset   = 0u * progSize;  // Start at the beginning of m_sbtBuffer
    vk::DeviceSize missOffset     = 1u * progSize;  // Jump over raygen
    vk::DeviceSize hitGroupOffset = 3u * progSize;  // Jump over the previous shaders
    vk::DeviceSize sbtSize        = progSize * (vk::DeviceSize)m_rtShaderGroups.size();

    // m_sbtBuffer holds all the shader handles: raygen, n-miss, hit...
    const vk::StridedBufferRegionKHR raygenShaderBindingTable = {m_rtSBTBuffer.handle, rayGenOffset,
                                                                 progSize, sbtSize};
    const vk::StridedBufferRegionKHR missShaderBindingTable   = {m_rtSBTBuffer.handle, missOffset,
                                                               progSize, sbtSize};
    const vk::StridedBufferRegionKHR hitShaderBindingTable = {m_rtSBTBuffer.handle, hitGroupOffset,
                                                              progSize, sbtSize};
    const vk::StridedBufferRegionKHR callableShaderBindingTable;
    cmdBuf.traceRaysKHR(&raygenShaderBindingTable, &missShaderBindingTable, &hitShaderBindingTable,
                        &callableShaderBindingTable,                  //
                        window_size_.width, window_size_.height, 1);  //

    m_debug.endLabel(cmdBuf);
}

void HelloVulkan::UpdateFrame()
{
    static glm::mat4 ref_camera_matrix;

    auto& m = camera_navigator_->ViewMatrix();
    if (memcmp(&ref_camera_matrix[0][0], &m[0][0], sizeof(m)) != 0) {
        ResetFrame();
        ref_camera_matrix = m;
        // LOG(INFO) << "view matrix cache updated";
    }
    m_rtPushConstants.frame += 1;
}

void HelloVulkan::ResetFrame()
{
    m_rtPushConstants.frame = -1;
}
=======
void HelloVulkan::raytrace(const vk::CommandBuffer& cmdBuf, const nvmath::vec4f& clearColor)
{
  updateFrame();

  m_debug.beginLabel(cmdBuf, "Ray trace");
  // Initializing push constant values
  m_rtPushConstants.clearColor     = clearColor;
  m_rtPushConstants.lightPosition  = m_pushConstant.lightPosition;
  m_rtPushConstants.lightIntensity = m_pushConstant.lightIntensity;
  m_rtPushConstants.lightType      = m_pushConstant.lightType;

  cmdBuf.bindPipeline(vk::PipelineBindPoint::eRayTracingKHR, m_rtPipeline);
  cmdBuf.bindDescriptorSets(vk::PipelineBindPoint::eRayTracingKHR, m_rtPipelineLayout, 0,
                            {m_rtDescSet, m_descSet}, {});
  cmdBuf.pushConstants<RtPushConstant>(m_rtPipelineLayout,
                                       vk::ShaderStageFlagBits::eRaygenKHR
                                           | vk::ShaderStageFlagBits::eClosestHitKHR
                                           | vk::ShaderStageFlagBits::eMissKHR,
                                       0, m_rtPushConstants);

  // Size of a program identifier
  uint32_t groupSize =
      nvh::align_up(m_rtProperties.shaderGroupHandleSize, m_rtProperties.shaderGroupBaseAlignment);
  uint32_t          groupStride = groupSize;
  vk::DeviceAddress sbtAddress  = m_device.getBufferAddress({m_rtSBTBuffer.buffer});

  using Stride = vk::StridedDeviceAddressRegionKHR;
  std::array<Stride, 4> strideAddresses{
      Stride{sbtAddress + 0u * groupSize, groupStride, groupSize * 1},  // raygen
      Stride{sbtAddress + 1u * groupSize, groupStride, groupSize * 2},  // miss
      Stride{sbtAddress + 3u * groupSize, groupStride, groupSize * 1},  // hit
      Stride{0u, 0u, 0u}};                                              // callable

  cmdBuf.traceRaysKHR(&strideAddresses[0], &strideAddresses[1], &strideAddresses[2],
                      &strideAddresses[3],  //
                      m_size.width, m_size.height,
                      1);  //


  m_debug.endLabel(cmdBuf);
}

//--------------------------------------------------------------------------------------------------
// If the camera matrix has changed, resets the frame.
// otherwise, increments frame.
//
void HelloVulkan::updateFrame()
{
  static nvmath::mat4f refCamMatrix;
  static float         refFov{CameraManip.getFov()};

  const auto& m   = CameraManip.getMatrix();
  const auto  fov = CameraManip.getFov();

  if(memcmp(&refCamMatrix.a00, &m.a00, sizeof(nvmath::mat4f)) != 0 || refFov != fov)
  {
    resetFrame();
    refCamMatrix = m;
    refFov       = fov;
  }
  m_rtPushConstants.frame++;
}

void HelloVulkan::resetFrame()
{
  m_rtPushConstants.frame = -1;
}
>>>>>>> b7ba7d4fdc4128cefb7ab1d6097cc6d21c37d817
