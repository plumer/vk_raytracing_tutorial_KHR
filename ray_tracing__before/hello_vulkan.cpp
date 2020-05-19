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

#define STB_IMAGE_IMPLEMENTATION
#include "fileformats/stb_image.h"
#include "obj_loader.h"

#include "hello_vulkan.h"
#include "nvh//cameramanipulator.hpp"
#include "nvvk/descriptorsets_vk.hpp"
#include "nvvk/pipeline_vk.hpp"

#include "nvh/fileoperations.hpp"
#include "nvvk/commands_vk.hpp"
#include "nvvk/renderpasses_vk.hpp"
#include "nvvk/shaders_vk.hpp"

namespace {
template <typename T>
uint32_t cast_u32(T v) {
  static_assert(std::is_arithmetic_v<T>, "Can't cast a non-arithmetic to u32");
  return static_cast<uint32_t>(v);
}
}


// Holding the camera matrices
struct CameraMatrices
{
  nvmath::mat4f view;
  nvmath::mat4f proj;
  nvmath::mat4f viewInverse;
};

//--------------------------------------------------------------------------------------------------
// Keep the handle on the device
// Initialize the tool to do all our allocations: buffers, images
//
void HelloVulkan::setup(const vk::Instance&       instance,
                        const vk::Device&         device,
                        const vk::PhysicalDevice& physicalDevice,
                        uint32_t                  queueFamily)
{
  AppBase::setup(instance, device, physicalDevice, queueFamily);
  m_alloc.init(device, physicalDevice);
  m_debug.setup(m_device);
}

//--------------------------------------------------------------------------------------------------
// Called at each frame to update the camera matrix
//
void HelloVulkan::updateUniformBuffer()
{
  const float aspectRatio = m_size.width / static_cast<float>(m_size.height);

  CameraMatrices ubo = {};
  ubo.view           = CameraManip.getMatrix();
  ubo.proj           = nvmath::perspectiveVK(CameraManip.getFov(), aspectRatio, 0.1f, 1000.0f);
  //ubo.proj[1][1] *= -1;  // Inverting Y for Vulkan
  ubo.viewInverse = nvmath::invert(ubo.view);
  void* data      = m_device.mapMemory(m_cameraMat.allocation, 0, sizeof(ubo));
  memcpy(data, &ubo, sizeof(ubo));
  m_device.unmapMemory(m_cameraMat.allocation);
}

//--------------------------------------------------------------------------------------------------
// Describing the layout pushed when rendering
//
void HelloVulkan::createDescriptorSetLayout()
{
  using vkDS     = vk::DescriptorSetLayoutBinding;
  using vkDT     = vk::DescriptorType;
  using vkSS     = vk::ShaderStageFlagBits;
  uint32_t nbTxt = static_cast<uint32_t>(m_textures.size());
  uint32_t nbObj = static_cast<uint32_t>(m_objModel.size());

  // Camera matrices (binding = 0)
  m_descSetLayoutBind.addBinding(vkDS(0, vkDT::eUniformBuffer, 1, vkSS::eVertex | vkSS::eRaygenKHR));
  // Materials (binding = 1)
  m_descSetLayoutBind.addBinding(
      vkDS(1, vkDT::eStorageBuffer, nbObj, vkSS::eVertex | vkSS::eFragment | vkSS::eClosestHitKHR));
  // Scene description (binding = 2)
  m_descSetLayoutBind.addBinding(  //
      vkDS(2, vkDT::eStorageBuffer, 1, vkSS::eVertex | vkSS::eFragment | vkSS::eClosestHitKHR));
  // Textures (binding = 3)
  m_descSetLayoutBind.addBinding(
      vkDS(3, vkDT::eCombinedImageSampler, nbTxt, vkSS::eFragment | vkSS::eClosestHitKHR));

  // Materials (binding = 4)
  m_descSetLayoutBind.addBinding(
      vkDS(4, vkDT::eStorageBuffer, nbObj, vkSS::eFragment | vkSS::eClosestHitKHR));

  // Storing vertices (binding = 5)
  m_descSetLayoutBind.addBinding(vkDS(5, vkDT::eStorageBuffer, nbObj, vkSS::eClosestHitKHR));

  // Storing indices (binding = 6)
  m_descSetLayoutBind.addBinding(vkDS(6, vkDT::eStorageBuffer, nbObj, vkSS::eClosestHitKHR));


  m_descSetLayout = m_descSetLayoutBind.createLayout(m_device);
  m_descPool      = m_descSetLayoutBind.createPool(m_device, 1);
  m_descSet       = nvvk::allocateDescriptorSet(m_device, m_descPool, m_descSetLayout);
}

//--------------------------------------------------------------------------------------------------
// Setting up the buffers in the descriptor set
//
void HelloVulkan::updateDescriptorSet()
{
  std::vector<vk::WriteDescriptorSet> writes;

  // Camera matrices and scene description
  vk::DescriptorBufferInfo dbiUnif{m_cameraMat.buffer, 0, VK_WHOLE_SIZE};
  writes.emplace_back(m_descSetLayoutBind.makeWrite(m_descSet, 0, &dbiUnif));
  vk::DescriptorBufferInfo dbiSceneDesc{m_sceneDesc.buffer, 0, VK_WHOLE_SIZE};
  writes.emplace_back(m_descSetLayoutBind.makeWrite(m_descSet, 2, &dbiSceneDesc));

  // All material buffers, 1 buffer per OBJ
  std::vector<vk::DescriptorBufferInfo> dbiMat;
  std::vector<vk::DescriptorBufferInfo> dbiMatIdx;
  std::vector<vk::DescriptorBufferInfo> dbi_vertices, dbi_indices;
  for(size_t i = 0; i < m_objModel.size(); ++i)
  {
    dbiMat.push_back({m_objModel[i].matColorBuffer.buffer, 0, VK_WHOLE_SIZE});
    dbiMatIdx.push_back({m_objModel[i].matIndexBuffer.buffer, 0, VK_WHOLE_SIZE});
    dbi_vertices.push_back({m_objModel[i].vertexBuffer.buffer, 0, VK_WHOLE_SIZE});
    dbi_indices.push_back({m_objModel[i].indexBuffer.buffer, 0, VK_WHOLE_SIZE});
  }
  writes.emplace_back(m_descSetLayoutBind.makeWriteArray(m_descSet, 1, dbiMat.data()));
  writes.emplace_back(m_descSetLayoutBind.makeWriteArray(m_descSet, 4, dbiMatIdx.data()));
  writes.emplace_back(m_descSetLayoutBind.makeWriteArray(m_descSet, 5, dbi_vertices.data()));
  writes.emplace_back(m_descSetLayoutBind.makeWriteArray(m_descSet, 6, dbi_indices.data()));

  // All texture samplers
  std::vector<vk::DescriptorImageInfo> diit;
  for(size_t i = 0; i < m_textures.size(); ++i)
  {
    diit.push_back(m_textures[i].descriptor);
  }
  writes.emplace_back(m_descSetLayoutBind.makeWriteArray(m_descSet, 3, diit.data()));

  // Writing the information
  m_device.updateDescriptorSets(static_cast<uint32_t>(writes.size()), writes.data(), 0, nullptr);
}


//--------------------------------------------------------------------------------------------------
// Creating the pipeline layout
//
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
  gpb.addShader(nvh::loadFile("shaders/vert_shader.vert.spv", true, paths), vkSS::eVertex);
  gpb.addShader(nvh::loadFile("shaders/frag_shader.frag.spv", true, paths), vkSS::eFragment);
  gpb.addBindingDescription({0, sizeof(VertexObj)});
  gpb.addAttributeDescriptions(std::vector<vk::VertexInputAttributeDescription>{
      {0, 0, vk::Format::eR32G32B32Sfloat, offsetof(VertexObj, pos)},
      {1, 0, vk::Format::eR32G32B32Sfloat, offsetof(VertexObj, nrm)},
      {2, 0, vk::Format::eR32G32B32Sfloat, offsetof(VertexObj, color)},
      {3, 0, vk::Format::eR32G32Sfloat, offsetof(VertexObj, texCoord)}});

  m_graphicsPipeline = gpb.createPipeline();
  m_debug.setObjectName(m_graphicsPipeline, "Graphics");
}

//--------------------------------------------------------------------------------------------------
// Loading the OBJ file and setting up all buffers
//
void HelloVulkan::loadModel(const std::string& filename, nvmath::mat4f transform)
{
  using vkBU = vk::BufferUsageFlagBits;

  ObjLoader loader;
  loader.loadModel(filename);

  // Converting from Srgb to linear
  for(auto& m : loader.m_materials)
  {
    m.ambient  = nvmath::pow(m.ambient, 2.2f);
    m.diffuse  = nvmath::pow(m.diffuse, 2.2f);
    m.specular = nvmath::pow(m.specular, 2.2f);
  }

  ObjInstance instance;
  instance.objIndex    = static_cast<uint32_t>(m_objModel.size());
  instance.transform   = transform;
  instance.transformIT = nvmath::transpose(nvmath::invert(transform));
  instance.txtOffset   = static_cast<uint32_t>(m_textures.size());

  ObjModel model;
  model.nbIndices  = static_cast<uint32_t>(loader.m_indices.size());
  model.nbVertices = static_cast<uint32_t>(loader.m_vertices.size());

  // Create the buffers on Device and copy vertices, indices and materials
  nvvk::CommandPool cmdBufGet(m_device, m_graphicsQueueIndex);
  vk::CommandBuffer cmdBuf = cmdBufGet.createCommandBuffer();
  // VKRay: adds buffer usages as storage buffers.
  model.vertexBuffer       = m_alloc.createBuffer(cmdBuf, loader.m_vertices, vkBU::eVertexBuffer | vkBU::eStorageBuffer);
  model.indexBuffer        = m_alloc.createBuffer(cmdBuf, loader.m_indices, vkBU::eIndexBuffer | vkBU::eStorageBuffer);
  model.matColorBuffer     = m_alloc.createBuffer(cmdBuf, loader.m_materials, vkBU::eStorageBuffer);
  model.matIndexBuffer     = m_alloc.createBuffer(cmdBuf, loader.m_matIndx, vkBU::eStorageBuffer);
  // Creates all textures found
  createTextureImages(cmdBuf, loader.m_textures);
  cmdBufGet.submitAndWait(cmdBuf);
  m_alloc.finalizeAndReleaseStaging();

  std::string objNb = std::to_string(instance.objIndex);
  m_debug.setObjectName(model.vertexBuffer.buffer, (std::string("vertex_" + objNb).c_str()));
  m_debug.setObjectName(model.indexBuffer.buffer, (std::string("index_" + objNb).c_str()));
  m_debug.setObjectName(model.matColorBuffer.buffer, (std::string("mat_" + objNb).c_str()));
  m_debug.setObjectName(model.matIndexBuffer.buffer, (std::string("matIdx_" + objNb).c_str()));

  m_objModel.emplace_back(model);
  m_objInstance.emplace_back(instance);
}


//--------------------------------------------------------------------------------------------------
// Creating the uniform buffer holding the camera matrices
// - Buffer is host visible
//
void HelloVulkan::createUniformBuffer()
{
  using vkBU = vk::BufferUsageFlagBits;
  using vkMP = vk::MemoryPropertyFlagBits;

  m_cameraMat = m_alloc.createBuffer(sizeof(CameraMatrices), vkBU::eUniformBuffer,
                                     vkMP::eHostVisible | vkMP::eHostCoherent);
  m_debug.setObjectName(m_cameraMat.buffer, "cameraMat");
}

//--------------------------------------------------------------------------------------------------
// Create a storage buffer containing the description of the scene elements
// - Which geometry is used by which instance
// - Transformation
// - Offset for texture
//
void HelloVulkan::createSceneDescriptionBuffer()
{
  using vkBU = vk::BufferUsageFlagBits;
  nvvk::CommandPool cmdGen(m_device, m_graphicsQueueIndex);

  auto cmdBuf = cmdGen.createCommandBuffer();
  m_sceneDesc = m_alloc.createBuffer(cmdBuf, m_objInstance, vkBU::eStorageBuffer);
  cmdGen.submitAndWait(cmdBuf);
  m_alloc.finalizeAndReleaseStaging();
  m_debug.setObjectName(m_sceneDesc.buffer, "sceneDesc");
}

//--------------------------------------------------------------------------------------------------
// Creating all textures and samplers
//
void HelloVulkan::createTextureImages(const vk::CommandBuffer&        cmdBuf,
                                      const std::vector<std::string>& textures)
{
  using vkIU = vk::ImageUsageFlagBits;

  vk::SamplerCreateInfo samplerCreateInfo{
      {}, vk::Filter::eLinear, vk::Filter::eLinear, vk::SamplerMipmapMode::eLinear};
  samplerCreateInfo.setMaxLod(FLT_MAX);
  vk::Format format = vk::Format::eR8G8B8A8Srgb;

  // If no textures are present, create a dummy one to accommodate the pipeline layout
  if(textures.empty() && m_textures.empty())
  {
    nvvk::Texture texture;

    std::array<uint8_t, 4> color{255u, 255u, 255u, 255u};
    vk::DeviceSize         bufferSize      = sizeof(color);
    auto                   imgSize         = vk::Extent2D(1, 1);
    auto                   imageCreateInfo = nvvk::makeImage2DCreateInfo(imgSize, format);

    // Creating the dummy texure
    nvvk::Image image = m_alloc.createImage(cmdBuf, bufferSize, color.data(), imageCreateInfo);
    vk::ImageViewCreateInfo ivInfo = nvvk::makeImageViewCreateInfo(image.image, imageCreateInfo);
    texture                        = m_alloc.createTexture(image, ivInfo, samplerCreateInfo);

    // The image format must be in VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL
    nvvk::cmdBarrierImageLayout(cmdBuf, texture.image, vk::ImageLayout::eUndefined,
                                vk::ImageLayout::eShaderReadOnlyOptimal);
    m_textures.push_back(texture);
  }
  else
  {
    // Uploading all images
    for(const auto& texture : textures)
    {
      std::stringstream o;
      int               texWidth, texHeight, texChannels;
      o << "media/textures/" << texture;
      std::string txtFile = nvh::findFile(o.str(), defaultSearchPaths);

      stbi_uc* pixels =
          stbi_load(txtFile.c_str(), &texWidth, &texHeight, &texChannels, STBI_rgb_alpha);

      // Handle failure
      if(!pixels)
      {
        texWidth = texHeight = 1;
        texChannels          = 4;
        std::array<uint8_t, 4> color{255u, 0u, 255u, 255u};
        pixels = reinterpret_cast<stbi_uc*>(color.data());
      }

      vk::DeviceSize bufferSize = static_cast<uint64_t>(texWidth) * texHeight * sizeof(uint8_t) * 4;
      auto           imgSize    = vk::Extent2D(texWidth, texHeight);
      auto imageCreateInfo = nvvk::makeImage2DCreateInfo(imgSize, format, vkIU::eSampled, true);

      {
        nvvk::ImageDedicated image =
            m_alloc.createImage(cmdBuf, bufferSize, pixels, imageCreateInfo);
        nvvk::cmdGenerateMipmaps(cmdBuf, image.image, format, imgSize, imageCreateInfo.mipLevels);
        vk::ImageViewCreateInfo ivInfo =
            nvvk::makeImageViewCreateInfo(image.image, imageCreateInfo);
        nvvk::Texture texture = m_alloc.createTexture(image, ivInfo, samplerCreateInfo);

        m_textures.push_back(texture);
      }
    }
  }
}

//--------------------------------------------------------------------------------------------------
// Destroying all allocations
//
void HelloVulkan::destroyResources()
{
    // VKRay
    m_rt_builder.destroy();
    m_alloc.destroy(m_rt_SBT_buffer);
    m_device.destroy(m_rt_pipeline);
    m_device.destroy(m_rt_pipeline_layout);
    m_device.destroy(m_rt_descriptor_pool);
    m_device.destroy(m_rt_descriptor_set_layout);
  m_device.destroy(m_graphicsPipeline);
  m_device.destroy(m_pipelineLayout);
  m_device.destroy(m_descPool);
  m_device.destroy(m_descSetLayout);
  m_alloc.destroy(m_cameraMat);
  m_alloc.destroy(m_sceneDesc);

  for(auto& m : m_objModel)
  {
    m_alloc.destroy(m.vertexBuffer);
    m_alloc.destroy(m.indexBuffer);
    m_alloc.destroy(m.matColorBuffer);
    m_alloc.destroy(m.matIndexBuffer);
  }

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
}

//--------------------------------------------------------------------------------------------------
// Drawing the scene in raster mode
//
void HelloVulkan::rasterize(const vk::CommandBuffer& cmdBuf)
{
  using vkPBP = vk::PipelineBindPoint;
  using vkSS  = vk::ShaderStageFlagBits;
  vk::DeviceSize offset{0};

  m_debug.beginLabel(cmdBuf, "Rasterize");

  // Dynamic Viewport
  cmdBuf.setViewport(0, {vk::Viewport(0, 0, (float)m_size.width, (float)m_size.height, 0, 1)});
  cmdBuf.setScissor(0, {{{0, 0}, {m_size.width, m_size.height}}});


  // Drawing all triangles
  cmdBuf.bindPipeline(vkPBP::eGraphics, m_graphicsPipeline);
  cmdBuf.bindDescriptorSets(vkPBP::eGraphics, m_pipelineLayout, 0, {m_descSet}, {});
  for(int i = 0; i < m_objInstance.size(); ++i)
  {
    auto& inst                = m_objInstance[i];
    auto& model               = m_objModel[inst.objIndex];
    m_pushConstant.instanceId = i;  // Telling which instance is drawn
    cmdBuf.pushConstants<ObjPushConstant>(m_pipelineLayout, vkSS::eVertex | vkSS::eFragment, 0,
                                          m_pushConstant);

    cmdBuf.bindVertexBuffers(0, {model.vertexBuffer.buffer}, {offset});
    cmdBuf.bindIndexBuffer(model.indexBuffer.buffer, 0, vk::IndexType::eUint32);
    cmdBuf.drawIndexed(model.nbIndices, 1, 0, 0, 0);
  }
  m_debug.endLabel(cmdBuf);
}

//--------------------------------------------------------------------------------------------------
// Handling resize of the window
//
void HelloVulkan::onResize(int /*w*/, int /*h*/)
{
  createOffscreenRender();
  updatePostDescriptorSet();
  update_rt_descriptor_set();
}


//////////////////////////////////////////////////////////////////////////
// Post-processing
//////////////////////////////////////////////////////////////////////////


//--------------------------------------------------------------------------------------------------
// Creating an offscreen frame buffer and the associated render pass
//
void HelloVulkan::createOffscreenRender()
{
  m_alloc.destroy(m_offscreenColor);
  m_alloc.destroy(m_offscreenDepth);

  // Creating the color image
  {
    auto colorCreateInfo = nvvk::makeImage2DCreateInfo(m_size, m_offscreenColorFormat,
                                                       vk::ImageUsageFlagBits::eColorAttachment
                                                           | vk::ImageUsageFlagBits::eSampled
                                                           | vk::ImageUsageFlagBits::eStorage);


    nvvk::ImageDedicated    image  = m_alloc.createImage(colorCreateInfo);
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
}

//--------------------------------------------------------------------------------------------------
// The pipeline is how things are rendered, which shaders, type of primitives, depth test and more
//
void HelloVulkan::createPostPipeline()
{
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
  pipelineGenerator.addShader(nvh::loadFile("shaders/passthrough.vert.spv", true, paths),
                              vk::ShaderStageFlagBits::eVertex);
  pipelineGenerator.addShader(nvh::loadFile("shaders/post.frag.spv", true, paths),
                              vk::ShaderStageFlagBits::eFragment);
  pipelineGenerator.rasterizationState.setCullMode(vk::CullModeFlagBits::eNone);
  m_postPipeline = pipelineGenerator.createPipeline();
  m_debug.setObjectName(m_postPipeline, "post");
}

//--------------------------------------------------------------------------------------------------
// The descriptor layout is the description of the data that is passed to the vertex or the
// fragment program.
//
void HelloVulkan::createPostDescriptor()
{
  using vkDS = vk::DescriptorSetLayoutBinding;
  using vkDT = vk::DescriptorType;
  using vkSS = vk::ShaderStageFlagBits;

  m_postDescSetLayoutBind.addBinding(vkDS(0, vkDT::eCombinedImageSampler, 1, vkSS::eFragment));
  m_postDescSetLayout = m_postDescSetLayoutBind.createLayout(m_device);
  m_postDescPool      = m_postDescSetLayoutBind.createPool(m_device);
  m_postDescSet       = nvvk::allocateDescriptorSet(m_device, m_postDescPool, m_postDescSetLayout);
}


//--------------------------------------------------------------------------------------------------
// Update the output
//
void HelloVulkan::updatePostDescriptorSet()
{
  vk::WriteDescriptorSet writeDescriptorSets =
      m_postDescSetLayoutBind.makeWrite(m_postDescSet, 0, &m_offscreenColor.descriptor);
  m_device.updateDescriptorSets(writeDescriptorSets, nullptr);
}

//--------------------------------------------------------------------------------------------------
// Draw a full screen quad with the attached image
//
void HelloVulkan::drawPost(vk::CommandBuffer cmdBuf)
{
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
}

void HelloVulkan::init_ray_tracing() {
  auto properties = m_physicalDevice.getProperties2<vk::PhysicalDeviceProperties2,
                                                    vk::PhysicalDeviceRayTracingPropertiesKHR>();
  m_rt_properties = properties.get<vk::PhysicalDeviceRayTracingPropertiesKHR>();

  std::cout << __FUNCTION__ << " | rt properties: \n"
            << "\tmaximum recursion depth = " << m_rt_properties.maxRecursionDepth << '\n'
            << "\tshader group handle size = " << m_rt_properties.shaderGroupHandleSize << '\n';

  m_rt_builder.setup(m_device, &m_alloc, m_graphicsQueueIndex);
}

nvvk::RaytracingBuilderKHR::Blas HelloVulkan::object_to_vkGeometryKHR(const ObjModel& model)
{
    // Sets up the creation info of AS.
    vk::AccelerationStructureCreateGeometryTypeInfoKHR as_create;
    as_create.setGeometryType(vk::GeometryTypeKHR::eTriangles);
    as_create.setIndexType(vk::IndexType::eUint32);
    as_create.setVertexFormat(vk::Format::eR32G32B32Sfloat);
    as_create.setMaxPrimitiveCount(model.nbIndices / 3);
    as_create.setMaxVertexCount(model.nbVertices);
    as_create.setAllowsTransforms(VK_FALSE);    // No adding transformation matrices.

    // Building part.
    vk::DeviceAddress vertex_address = m_device.getBufferAddress(
        {model.indexBuffer.buffer}, vk::DispatchLoaderStatic());
    vk::DeviceAddress index_address = m_device.getBufferAddress(
        {model.indexBuffer.buffer}, vk::DispatchLoaderStatic());

    auto triangles = vk::AccelerationStructureGeometryTrianglesDataKHR()
                         .setVertexFormat(as_create.vertexFormat)
                         .setVertexData(vertex_address)
                         .setVertexStride(sizeof(VertexObj))
                         .setIndexType(as_create.indexType)
                         .setIndexData(index_address)
                         .setTransformData({});

    // Sets up the build info of the acceleration structure.
    auto as_geometry = vk::AccelerationStructureGeometryKHR()
                           .setGeometryType(as_create.geometryType)
                           .setFlags(vk::GeometryFlagBitsKHR::eOpaque);
    as_geometry.geometry.setTriangles(triangles);

    // Sets the primitive.
    auto offset = vk::AccelerationStructureBuildOffsetInfoKHR()
                      .setFirstVertex(0)
                      .setPrimitiveCount(as_create.maxPrimitiveCount)
                      .setPrimitiveOffset(0)
                      .setTransformOffset(0);


    nvvk::RaytracingBuilderKHR::Blas blas;
    blas.asGeometry.emplace_back(as_geometry);
    blas.asCreateGeometryInfo.emplace_back(as_create);
    blas.asBuildOffsetInfo.emplace_back(offset);

    return blas;
}

void HelloVulkan::create_bottom_level_AS() {
    // BLAS: storing each primitive in a geometry.
    std::vector<nvvk::RaytracingBuilderKHR::Blas> all_blas;
    all_blas.reserve(m_objModel.size());
    for (const auto& obj : m_objModel) {
      auto blas = object_to_vkGeometryKHR(obj);
      all_blas.emplace_back(blas);
    }
    m_rt_builder.buildBlas(all_blas, vk::BuildAccelerationStructureFlagBitsKHR::ePreferFastTrace);
}

void HelloVulkan::create_top_level_AS() {
  std::vector<nvvk::RaytracingBuilderKHR::Instance> tlas_instances;
  tlas_instances.reserve(m_objInstance.size());
  
  for (int i = 0; i < static_cast<int>(m_objInstance.size()); ++i) {
      nvvk::RaytracingBuilderKHR::Instance ray_instance;
      ray_instance.transform = m_objInstance[i].transform;
      ray_instance.instanceId = i;
      ray_instance.blasId = m_objInstance[i].objIndex;
      ray_instance.hitGroupId = 0; // Uses the same hit group for all objects.
      ray_instance.flags = VK_GEOMETRY_INSTANCE_TRIANGLE_FACING_CULL_DISABLE_BIT_KHR;
      tlas_instances.emplace_back(ray_instance);
  }
  m_rt_builder.buildTlas(tlas_instances, vk::BuildAccelerationStructureFlagBitsKHR::ePreferFastTrace);
}

void HelloVulkan::create_rt_descriptor_set() {
    using vkDSType = vk::DescriptorType;
    using StageBits = vk::ShaderStageFlagBits;
    using DescBinding = vk::DescriptorSetLayoutBinding;

    // Binding #0: TLAS
    m_rt_descriptor_set_layout_bind.addBinding(
        DescBinding(0, vkDSType::eAccelerationStructureKHR, 1,
                    StageBits::eRaygenKHR | StageBits::eClosestHitKHR));
    // Binding #1: Output image.
    m_rt_descriptor_set_layout_bind.addBinding(
        DescBinding(1, vkDSType::eStorageImage, 1, StageBits::eRaygenKHR));

    m_rt_descriptor_pool = m_rt_descriptor_set_layout_bind.createPool(m_device);
    m_rt_descriptor_set_layout = m_rt_descriptor_set_layout_bind.createLayout(m_device);
    m_rt_descriptor_set =
        m_device.allocateDescriptorSets({m_rt_descriptor_pool, 1, &m_rt_descriptor_set_layout})[0];

    vk::AccelerationStructureKHR tlas = m_rt_builder.getAccelerationStructure();
    vk::WriteDescriptorSetAccelerationStructureKHR descriptor_AS_info;
    descriptor_AS_info.setAccelerationStructureCount(1).setPAccelerationStructures(&tlas);

    vk::DescriptorImageInfo image_info{
      {}, m_offscreenColor.descriptor.imageView, vk::ImageLayout::eGeneral
    };

    std::vector<vk::WriteDescriptorSet> writes;
    writes.emplace_back(m_rt_descriptor_set_layout_bind.makeWrite(m_rt_descriptor_set, 0, &descriptor_AS_info));
    writes.emplace_back(m_rt_descriptor_set_layout_bind.makeWrite(m_rt_descriptor_set, 1, &image_info));
    m_device.updateDescriptorSets(static_cast<uint32_t>(writes.size()), writes.data(), 0, nullptr);
}

void HelloVulkan::update_rt_descriptor_set() {
    // Updates the image reference.
    using vkDSType = vk::DescriptorType;
    // Binding #1: output buffer
    auto image_info = vk::DescriptorImageInfo().setImageView(m_offscreenColor.descriptor.imageView)
                          .setImageLayout(vk::ImageLayout::eGeneral);
    auto write_DS = vk::WriteDescriptorSet(m_rt_descriptor_set, 1, 0, 1, vkDSType::eStorageImage, &image_info);
    m_device.updateDescriptorSets(write_DS, nullptr);
}

void HelloVulkan::create_rt_pipeline()
{
  std::vector<std::string> paths = defaultSearchPaths;
  vk::ShaderModule         raygen_SM =
      nvvk::createShaderModule(m_device, nvh::loadFile("shaders/raytrace.rgen.spv", true, paths));
  vk::ShaderModule miss_SM =
      nvvk::createShaderModule(m_device, nvh::loadFile("shaders/raytrace.rmiss.spv", true, paths));
  vk::ShaderModule closest_hit_SM =
      nvvk::createShaderModule(m_device, nvh::loadFile("shaders/raytrace.rchit.spv", true, paths));

  std::vector<vk::PipelineShaderStageCreateInfo> stages_ci;
  stages_ci.emplace_back(vk::PipelineShaderStageCreateInfo({}, vk::ShaderStageFlagBits::eRaygenKHR,
                                                           raygen_SM, "main"));
  auto raygen_group_ci = vk::RayTracingShaderGroupCreateInfoKHR()
                             .setType(vk::RayTracingShaderGroupTypeKHR::eGeneral)
                             .setAnyHitShader(VK_SHADER_UNUSED_KHR)
                             .setClosestHitShader(VK_SHADER_UNUSED_KHR)
                             .setGeneralShader(VK_SHADER_UNUSED_KHR)
                             .setIntersectionShader(VK_SHADER_UNUSED_KHR)
                             .setGeneralShader(static_cast<uint32_t>(stages_ci.size() - 1));
  stages_ci.emplace_back(
      vk::PipelineShaderStageCreateInfo({}, vk::ShaderStageFlagBits::eMissKHR, miss_SM, "main"));
  auto miss_group_ci = vk::RayTracingShaderGroupCreateInfoKHR()
                           .setType(vk::RayTracingShaderGroupTypeKHR::eGeneral)
                           .setAnyHitShader(VK_SHADER_UNUSED_KHR)
                           .setClosestHitShader(VK_SHADER_UNUSED_KHR)
                           .setGeneralShader(VK_SHADER_UNUSED_KHR)
                           .setIntersectionShader(VK_SHADER_UNUSED_KHR)
                           .setGeneralShader(static_cast<uint32_t>(stages_ci.size() - 1));

  stages_ci.emplace_back(vk::PipelineShaderStageCreateInfo(
      {}, vk::ShaderStageFlagBits::eClosestHitKHR, closest_hit_SM, "main"));

  auto closest_hit_group_ci = vk::RayTracingShaderGroupCreateInfoKHR()
                                  .setType(vk::RayTracingShaderGroupTypeKHR::eTrianglesHitGroup)
                                  .setAnyHitShader(VK_SHADER_UNUSED_KHR)
                                  .setClosestHitShader(VK_SHADER_UNUSED_KHR)
                                  .setGeneralShader(VK_SHADER_UNUSED_KHR)
                                  .setIntersectionShader(VK_SHADER_UNUSED_KHR)
                                  .setGeneralShader(static_cast<uint32_t>(stages_ci.size() - 1));

  m_rt_shader_groups.push_back(raygen_group_ci);
  m_rt_shader_groups.push_back(miss_group_ci);
  m_rt_shader_groups.push_back(closest_hit_group_ci);

  // Sets up the pipeline layout which describes how the pipeline accesses external data.
  vk::PipelineLayoutCreateInfo pipeline_layout_ci;
  vk::PushConstantRange        push_constant(vk::ShaderStageFlagBits::eRaygenKHR
                                          | vk::ShaderStageFlagBits::eClosestHitKHR
                                          | vk::ShaderStageFlagBits::eMissKHR,
                                      0, sizeof(RtPushConstant));
  pipeline_layout_ci.setPushConstantRangeCount(1).setPPushConstantRanges(&push_constant);
  std::vector<vk::DescriptorSetLayout> rt_DS_layouts = {m_rt_descriptor_set_layout, m_descSetLayout};
  pipeline_layout_ci.setSetLayoutCount(2).setPSetLayouts(rt_DS_layouts.data());
  m_rt_pipeline_layout = m_device.createPipelineLayout(pipeline_layout_ci);

  // Creates the RT pipeline. Different from raster pipeline, RT pipeline can contain an arbitrary
  // number of stages depending on the number of active shaders in the scene.
  vk::RayTracingPipelineCreateInfoKHR rt_pipeline_ci;
  rt_pipeline_ci.setStageCount(static_cast<uint32_t>(stages_ci.size()))
      .setPStages(stages_ci.data())
      // Specifies how the shaders can be assembled into groups. RG or miss shader is a group by 
      // itself, but hit groups consists of up to 3 shaders (intersection, any-hit, closest-hit).
      // TODO: see documentation of VkRayTracingShaderGroupCreateInfo.
      // 1-raygen, n-miss, n-(hit[+anyhit+intersect]).
      .setGroupCount(static_cast<uint32_t>(m_rt_shader_groups.size()))
      .setPGroups(m_rt_shader_groups.data())
      .setMaxRecursionDepth(1).setLayout(m_rt_pipeline_layout);
  m_rt_pipeline = m_device.createRayTracingPipelineKHR(/*cache = */{}, rt_pipeline_ci).value;

  // ONce the pipeline has been created, we can discard the shader modules.
  m_device.destroyShaderModule(raygen_SM);
  m_device.destroyShaderModule(miss_SM);
  m_device.destroyShaderModule(closest_hit_SM);
}

void HelloVulkan::create_rt_shader_binding_table() {
  auto group_count = cast_u32(m_rt_shader_groups.size());
  // Size of a program identifier.
  uint32_t group_handle_size = m_rt_properties.shaderGroupHandleSize;

  // Fetches all shader handles used in the pipeline, so that they can be written in the SBT.
  uint32_t SBT_size = group_count * group_handle_size;
  std::vector<uint8_t> shader_handle_storage(SBT_size);
  m_device.getRayTracingShaderGroupHandlesKHR(m_rt_pipeline, 0, group_count, SBT_size,
                                              shader_handle_storage.data());

  // Writes the handles to SBT.
  nvvk::CommandPool gen_cmd_buffer(m_device, m_graphicsQueueIndex);
  vk::CommandBuffer cmd_buffer = gen_cmd_buffer.createCommandBuffer();

  m_rt_SBT_buffer = m_alloc.createBuffer(cmd_buffer, shader_handle_storage,
                                         vk::BufferUsageFlagBits::eRayTracingKHR);
  m_debug.setObjectName(m_rt_SBT_buffer.buffer, "SBT");

  gen_cmd_buffer.submitAndWait(cmd_buffer);
  m_alloc.finalizeAndReleaseStaging();
}

void HelloVulkan::ray_trace(const vk::CommandBuffer& cmd_buffer, const nvmath::vec4f& clear_color)
{
    using vkSSType = vk::ShaderStageFlagBits;
  m_debug.beginLabel(cmd_buffer, "Ray trace");
  // Initializes push constant values.
  m_rt_push_constants.clear_color     = clear_color;
  m_rt_push_constants.light_position  = m_pushConstant.lightPosition;
  m_rt_push_constants.light_intensity = m_pushConstant.lightIntensity;
  m_rt_push_constants.light_type      = m_pushConstant.lightType;

  cmd_buffer.bindPipeline(vk::PipelineBindPoint::eRayTracingKHR, m_rt_pipeline);
  cmd_buffer.bindDescriptorSets(vk::PipelineBindPoint::eRayTracingKHR, m_rt_pipeline_layout, 0,
                                {m_rt_descriptor_set, m_descSet}, /*dynamic_offset = */{});
  cmd_buffer.pushConstants<RtPushConstant>(
      m_rt_pipeline_layout, vkSSType::eRaygenKHR | vkSSType::eClosestHitKHR | vkSSType::eMissKHR, 0,
      m_rt_push_constants);

  // Tells the RT pipeline how to interpret SBT.
  // -------------------------------------------

  // Size of a program identifer.
  vk::DeviceSize program_size     = m_rt_properties.shaderGroupHandleSize;
  vk::DeviceSize raygen_offset    = 0u * program_size;  // Start at the beginning of sbt buffer.
  vk::DeviceSize raygen_stride    = program_size;
  vk::DeviceSize miss_offset      = 1u * program_size;  // Jump over raygen.
  vk::DeviceSize miss_stride      = program_size;
  vk::DeviceSize hit_group_offset = 2u * program_size; // Jump over the previous headers.
  vk::DeviceSize hit_group_stride = program_size;

  vk::DeviceSize SBT_size = program_size * (vk::DeviceSize)m_rt_shader_groups.size();

  auto raygen_SBT =
      vk::StridedBufferRegionKHR(m_rt_SBT_buffer.buffer, raygen_offset, raygen_stride, SBT_size);
  auto miss_SBT = raygen_SBT;
  miss_SBT.setOffset(miss_offset);
  auto hit_SBT = raygen_SBT;
  hit_SBT.setOffset(hit_group_offset);
  auto callable_SBT = vk::StridedBufferRegionKHR();

  cmd_buffer.traceRaysKHR(raygen_SBT, miss_SBT, hit_SBT, callable_SBT, m_size.width, m_size.height,
                          /* depth = */1);

  m_debug.endLabel(cmd_buffer);
}

