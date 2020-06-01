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
#pragma once

//#define NVVK_ALLOC_DEDICATED
#define NVVK_ALLOC_DMA
#include "nvvk/raytraceKHR_vk.hpp"
#include "nvvk/allocator_vk.hpp"
#include "nvvk/appbase_vkpp.hpp"
#include "nvvk/debug_util_vk.hpp"
#include "nvvk/descriptorsets_vk.hpp"

//--------------------------------------------------------------------------------------------------
// Simple rasterizer of OBJ objects
// - Each OBJ loaded are stored in an `ObjModel` and referenced by a `ObjInstance`
// - It is possible to have many `ObjInstance` referencing the same `ObjModel`
// - Rendering is done in an offscreen framebuffer
// - The image of the framebuffer is displayed in post-process in a full-screen quad
//
class HelloVulkan : public nvvk::AppBase
{
public:
  void setup(const vk::Instance&       instance,
             const vk::Device&         device,
             const vk::PhysicalDevice& physicalDevice,
             uint32_t                  queueFamily) override;
  void createDescriptorSetLayout();
  void createGraphicsPipeline();
  void loadModel(const std::string& filename, nvmath::mat4f transform = nvmath::mat4f(1));
  void updateDescriptorSet();
  void createUniformBuffer();
  void createSceneDescriptionBuffer();
  void createTextureImages(const vk::CommandBuffer&        cmdBuf,
                           const std::vector<std::string>& textures);
  void updateUniformBuffer();
  void onResize(int /*w*/, int /*h*/) override;
  void destroyResources();
  void rasterize(const vk::CommandBuffer& cmdBuff);

  // The OBJ model
  struct ObjModel
  {
    uint32_t   nbIndices{0};
    uint32_t   nbVertices{0};
    nvvk::Buffer vertexBuffer;    // Device buffer of all 'Vertex'
    nvvk::Buffer indexBuffer;     // Device buffer of the indices forming triangles
    nvvk::Buffer matColorBuffer;  // Device buffer of array of 'Wavefront material'
    nvvk::Buffer matIndexBuffer;  // Device buffer of array of 'Wavefront material'
  };

  // Instance of the OBJ
  struct ObjInstance
  {
    uint32_t      objIndex{0};     // Reference to the `m_objModel`
    uint32_t      txtOffset{0};    // Offset in `m_textures`
    nvmath::mat4f transform{1};    // Position of the instance
    nvmath::mat4f transformIT{1};  // Inverse transpose
  };

  // Information pushed at each draw call
  struct ObjPushConstant
  {
    nvmath::vec3f lightPosition{10.f, 15.f, 8.f};
    int           instanceId{0};  // To retrieve the transformation matrix
    float         lightIntensity{100.f};
    int           lightType{0};  // 0: point, 1: infinite
  };
  ObjPushConstant m_pushConstant;

  // Array of objects and instances in the scene
  std::vector<ObjModel>    m_objModel;
  std::vector<ObjInstance> m_objInstance;

  // Graphic pipeline
  vk::PipelineLayout          m_pipelineLayout;
  vk::Pipeline                m_graphicsPipeline;
  nvvk::DescriptorSetBindings m_descSetLayoutBind;
  vk::DescriptorPool          m_descPool;
  vk::DescriptorSetLayout     m_descSetLayout;
  vk::DescriptorSet           m_descSet;

  nvvk::Buffer               m_cameraMat;  // Device-Host of the camera matrices
  nvvk::Buffer               m_sceneDesc;  // Device buffer of the OBJ instances
  std::vector<nvvk::Texture> m_textures;   // vector of all textures of the scene

#if defined(NVVK_ALLOC_DEDICATED)
  nvvk::AllocatorDedicated m_alloc;  // Allocator for buffer, images, acceleration structures
#elif defined(NVVK_ALLOC_DMA)
  nvvk::AllocatorDma            m_alloc;
  nvvk::DeviceMemoryAllocator   m_mem_allocator;
  nvvk::StagingMemoryManagerDma m_staging;

#endif  // NVVK_ALLOC_DMA/DEDICATED

  nvvk::DebugUtil          m_debug;  // Utility to name objects


  // #Post
  void createOffscreenRender();
  void createPostPipeline();
  void createPostDescriptor();
  void updatePostDescriptorSet();
  void drawPost(vk::CommandBuffer cmdBuf);

  nvvk::DescriptorSetBindings m_postDescSetLayoutBind;
  vk::DescriptorPool          m_postDescPool;
  vk::DescriptorSetLayout     m_postDescSetLayout;
  vk::DescriptorSet           m_postDescSet;
  vk::Pipeline                m_postPipeline;
  vk::PipelineLayout          m_postPipelineLayout;
  vk::RenderPass              m_offscreenRenderPass;
  vk::Framebuffer             m_offscreenFramebuffer;
  nvvk::Texture               m_offscreenColor;
  vk::Format                  m_offscreenColorFormat{vk::Format::eR32G32B32A32Sfloat};
  nvvk::Texture               m_offscreenDepth;
  vk::Format                  m_offscreenDepthFormat{vk::Format::eD32Sfloat};

  // VKRay
  void init_ray_tracing();
  vk::PhysicalDeviceRayTracingPropertiesKHR m_rt_properties;
  nvvk::RaytracingBuilderKHR                m_rt_builder;

  // Converts an OBJ primitive to the ray tracing geometry used for the BLAS.
  nvvk::RaytracingBuilderKHR::Blas object_to_vkGeometryKHR(const ObjModel &model);

  // Generates a BLAS for each object.
  void create_bottom_level_AS();

  // TLAS is the entry point in the ray-tracing scene description, storing all
  // the instances.
  // 
  // An instance is a nvvk::RaytracingBuilder::Instance, storing its transform
  // matrix and the identifier of its corresponding BLAS. It also contains an 
  // instance identifier that will be available during shading 
  // (gl_InstanceCustomIndex), as well as the index of the hit group 
  // representing the shaders that will be invoked upon hitting the object.
  void create_top_level_AS();

  void create_rt_descriptor_set();
  nvvk::DescriptorSetBindings   m_rt_descriptor_set_layout_bind;
  vk::DescriptorPool            m_rt_descriptor_pool;
  vk::DescriptorSetLayout       m_rt_descriptor_set_layout;
  vk::DescriptorSet             m_rt_descriptor_set;

  // Updates the descriptors. This typically happens when resizing the window,
  // as the output image is recreated and needs to be re-linked to the DS.
  void update_rt_descriptor_set();

  void create_rt_pipeline();
  std::vector<vk::RayTracingShaderGroupCreateInfoKHR> m_rt_shader_groups;
  vk::PipelineLayout m_rt_pipeline_layout;
  vk::Pipeline m_rt_pipeline;
  struct RtPushConstant {
      nvmath::vec4f clear_color;
      nvmath::vec3f light_position;
      float         light_intensity = 0.0f;
      int           light_type      = 0;
  } m_rt_push_constants;

  // Shader binding table: blueprint of the RT process.
  // Indicates which raygen shader to start with, which miss shader to execute
  // if no intersections are found, and which hit-shader groups can be executed
  // for each instance.
  // Association between instances and shader groups is created when setting up
  // the geometry, e.g., hitGroupId in TLAS for each instance.

  // Gets all shader handles and writes them to a SBT buffer.
  void create_rt_shader_binding_table();
  nvvk::Buffer m_rt_SBT_buffer;

  // Executes the ray tracer.
  void ray_trace(const vk::CommandBuffer &cmd_buffer,
                 const nvmath::vec4f &clear_color);
};
