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
<<<<<<< HEAD

#include "types.h"

#define NVVK_ALLOC_DEDICATED
#include "nvvk/debug_util_vk.hpp"
#include "vk_appbase.h"
#include "vk_memory.h"
#include "vk_raytrace.hpp"
#include "vk_utils.h"
// #VKRay
#include "vk_raytrace.hpp"
#include "pbrt_scene.h"
#include <nvh/gltfscene.hpp>
=======
#include <vulkan/vulkan.hpp>

#define NVVK_ALLOC_DEDICATED
#include "nvvk/allocator_vk.hpp"
#include "nvvk/appbase_vkpp.hpp"
#include "nvvk/debug_util_vk.hpp"
#include "nvvk/descriptorsets_vk.hpp"

// #VKRay
#include "nvh/gltfscene.hpp"
#include "nvvk/raytraceKHR_vk.hpp"
>>>>>>> b7ba7d4fdc4128cefb7ab1d6097cc6d21c37d817

//--------------------------------------------------------------------------------------------------
// Simple rasterizer of OBJ objects
// - Each OBJ loaded are stored in an `ObjModel` and referenced by a `ObjInstance`
// - It is possible to have many `ObjInstance` referencing the same `ObjModel`
// - Rendering is done in an offscreen framebuffer
// - The image of the framebuffer is displayed in post-process in a full-screen quad
//
<<<<<<< HEAD
class HelloVulkan : public vkpbr::AppBase
{
  public:
    HelloVulkan() = default;
    void Setup(const vk::Instance& instance, const vk::Device& device,
               const vk::PhysicalDevice& physicalDevice, u32 queueFamily) override;
    void BuildDescriptorSetLayout();
    void BuildGraphicsPipeline();
    void LoadModel(const std::string& filename, glm::mat4 transform = glm::mat4(1));

    // Reads a gltf file and writes the scene data to buffers.
    void LoadGltfModel(const std::string& filename, glm::mat4 transform = glm::mat4(1.0f));

    void LoadPbrtModel(const std::string &filename);
    vkpbr::Scene pbr_scene;
    void UpdateDescriptorSet();
    void BuildUniformBuffer();
    void BuildSceneDescriptionBuffer();
    void BuildTextureImages(const vk::CommandBuffer&        cmdBuf,
                            const std::vector<std::string>& textures);

    void UpdateUniformBuffer();
    void WindowResizeCallback(int /*w*/, int /*h*/) override;
    void destroyResources();
    void rasterize(const vk::CommandBuffer& cmdBuff);

    // The OBJ model
    struct ObjModel {
        uint32_t                  nbIndices{0};
        uint32_t                  nbVertices{0};
        vkpbr::UniqueMemoryBuffer vertexBuffer;    // Device buffer of all 'Vertex'
        vkpbr::UniqueMemoryBuffer indexBuffer;     // Device buffer of the indices forming triangles
        vkpbr::UniqueMemoryBuffer matColorBuffer;  // Device buffer of array of 'Wavefront material'
        vkpbr::UniqueMemoryBuffer matIndexBuffer;  // Device buffer of array of 'Wavefront material'
    };


    // Instance of the OBJ
    struct ObjInstance {
        uint32_t  objIndex{0};     // Reference to the `m_objModel`
        uint32_t  txtOffset{0};    // Offset in `m_textures`
        glm::mat4 transform{1};    // Position of the instance
        glm::mat4 transformIT{1};  // Inverse transpose
        glm::mat4 dummy;
    };

    // Information pushed at each draw call
    struct ObjPushConstant {
        glm::vec3 lightPosition{10.f, 15.f, 8.f};
        int       instanceId{0};  // To retrieve the transformation matrix
        float     lightIntensity{100.f};
        int       lightType{0};  // 0: point, 1: infinite
        int       materialId = 0;
    };
    ObjPushConstant m_pushConstant;

    // Array of objects and instances in the scene
    std::vector<ObjModel>    m_objModel;
    std::vector<ObjInstance> m_objInstance;

    // Graphic pipeline
    vk::PipelineLayout           m_pipelineLayout;
    vk::Pipeline                 m_graphicsPipeline;
    vkpbr::DescriptorSetBindings DS_layout_bindings_;
    vk::DescriptorPool           m_descPool;
    vk::DescriptorSetLayout      m_descSetLayout;
    vk::DescriptorSet            m_descSet;

    vkpbr::UniqueMemoryBuffer m_cameraMat;  // Device-Host of the camera matrices
    vkpbr::UniqueMemoryBuffer m_sceneDesc;  // Device buffer of the OBJ instances
    // std::vector<nvvk::Texture> m_textures;   // vector of all textures of the scene
    std::vector<vkpbr::UniqueMemoryTexture> m_textures;

    // nvvk::AllocatorDedicated     m_alloc;  // Allocator for buffer, images, acceleration
    // structures
    vkpbr::UniqueMemoryAllocator allocator_;
    nvvk::DebugUtil              m_debug;  // Utility to name objects

    // #Post
    void createOffscreenRender();
    void createPostPipeline();
    void createPostDescriptor();
    void updatePostDescriptorSet();
    void drawPost(vk::CommandBuffer cmdBuf);

    vkpbr::DescriptorSetBindings post_DS_layout_bindings_;
    vk::DescriptorPool           m_postDescPool;
    vk::DescriptorSetLayout      m_postDescSetLayout;
    vk::DescriptorSet            m_postDescSet;
    vk::Pipeline                 m_postPipeline;
    vk::PipelineLayout           m_postPipelineLayout;
    vk::RenderPass               m_offscreenRenderPass;
    vk::Framebuffer              m_offscreenFramebuffer;
    vkpbr::UniqueMemoryTexture   m_offscreenColor;
    vk::Format                   m_offscreenColorFormat{vk::Format::eR32G32B32A32Sfloat};
    vkpbr::UniqueMemoryTexture   m_offscreenDepth;
    vk::Format                   m_offscreenDepthFormat{vk::Format::eD32Sfloat};

    // Binding indices for the regular descriptor set.
    enum DSBindings {
        kDsbCameraMatrices = 0,
        kDsbVertices,
        kDsbNormals,
        kDsbTexcoords,
        kDsbIndices,
        kDsbMaterials,
        kDsbMatrices,
        kDsbTextures
    };

    void WriteFramebuffer(const std::string &file_name);

    // #VKRay
    void                              initRayTracing();
    vkpbr::RaytracingBuilderKHR::Blas objectToVkGeometryKHR(const ObjModel& model);
    vkpbr::RaytracingBuilderKHR::Blas PrimitiveToGeometryKHR(const nvh::GltfPrimMesh& prim);

    // Creates bottom level accel structure for each model in m_objModel and glTF scene.
    void createBottomLevelAS();
    void createTopLevelAS();
    void createRtDescriptorSet();
    void updateRtDescriptorSet();
    void createRtPipeline();
    void createRtShaderBindingTable();
    void raytrace(const vk::CommandBuffer& cmdBuf, const glm::vec4& clearColor);

    // Increments the frame number if the camera view isn't moved.
    void UpdateFrame();
    void ResetFrame();

    struct RtPrimitiveLookup {
        u32 index_offset   = 0;
        u32 vertex_offset  = 0;
        i32 material_index = 0;
    };


    vk::PhysicalDeviceRayTracingPropertiesKHR m_rtProperties;
    vkpbr::RaytracingBuilderKHR               m_rtBuilder;

    // Binding indices for the ray tracing descriptor set.
    enum RtDSBindings { kRtDsbAccelStruct = 0, kRtDsbOutputImage, kRtDsbPrimInfo };
    vkpbr::DescriptorSetBindings rt_DS_layout_bindings_;
    vk::DescriptorPool           m_rtDescPool;
    vk::DescriptorSetLayout      m_rtDescSetLayout;
    vk::DescriptorSet            m_rtDescSet;

    enum RtPipelineStages {
        kRaygen = 0,
        kMiss,
        kShadowMiss,
        kClosestHit,
        // kClosestHit2,
        kNumStages
    };
    std::vector<vk::RayTracingShaderGroupCreateInfoKHR> m_rtShaderGroups;
    vk::PipelineLayout                                  m_rtPipelineLayout;
    vk::Pipeline                                        m_rtPipeline;

    vkpbr::UniqueMemoryBuffer m_rtSBTBuffer;

    struct GltfSceneData {
        vkpbr::UniqueMemoryBuffer vertex_buffer;
        vkpbr::UniqueMemoryBuffer index_buffer;
        vkpbr::UniqueMemoryBuffer normal_buffer;
        vkpbr::UniqueMemoryBuffer uv_buffer;
        vkpbr::UniqueMemoryBuffer mtl_buffer;
        vkpbr::UniqueMemoryBuffer matrix_buffer;
        vkpbr::UniqueMemoryBuffer rt_prim_lookup_buffer;
    } scene_data_;
    nvh::GltfScene gltf_scene_;
    // TODO: Reads a gltf model and loads all images as textures.
    void BuildTextureImages(const vk::CommandBuffer& cmd_buffer, tinygltf::Model& gltf_model);


    struct RtPushConstant {
        glm::vec4 clearColor;
        glm::vec3 lightPosition;
        float     lightIntensity     = 0;
        int       lightType          = 0;
        //int       selected_mtl_index = 0;
        //glm::vec3 selected_mtl_color{0.1f, 0.1f, 0.1f};
        //int       path_length = 5;
        int frame = 0;
    } m_rtPushConstants;


    struct Sphere {
        glm::vec3 center;
        float     radius;
    };

    struct Aabb {
        glm::vec3 minimum;
        glm::vec3 maximum;
    };
    vkpbr::RaytracingBuilderKHR::Blas sphereToVkGeometryKHR();
    std::vector<Sphere>               m_spheres;                // All spheres
    vkpbr::UniqueMemoryBuffer         m_spheresBuffer;          // Buffer holding the spheres
    vkpbr::UniqueMemoryBuffer         m_spheresAabbBuffer;      // Buffer of all Aabb
    vkpbr::UniqueMemoryBuffer         m_spheresMatColorBuffer;  // Multiple materials
    vkpbr::UniqueMemoryBuffer m_spheresMatIndexBuffer;  // Define which sphere uses which material
    void                      createSpheres();
=======
class HelloVulkan : public nvvk::AppBase
{
public:
  void setup(const vk::Instance&       instance,
             const vk::Device&         device,
             const vk::PhysicalDevice& physicalDevice,
             uint32_t                  queueFamily) override;
  void createDescriptorSetLayout();
  void createGraphicsPipeline();
  void loadScene(const std::string& filename);
  void updateDescriptorSet();
  void createUniformBuffer();
  void createTextureImages(const vk::CommandBuffer& cmdBuf, tinygltf::Model& gltfModel);
  void updateUniformBuffer(const vk::CommandBuffer& cmdBuf);
  void onResize(int /*w*/, int /*h*/) override;
  void destroyResources();
  void rasterize(const vk::CommandBuffer& cmdBuff);

  // Structure used for retrieving the primitive information in the closest hit
  // The gl_InstanceCustomIndexNV
  struct RtPrimitiveLookup
  {
    uint32_t indexOffset;
    uint32_t vertexOffset;
    int      materialIndex;
  };


  nvh::GltfScene m_gltfScene;
  nvvk::Buffer   m_vertexBuffer;
  nvvk::Buffer   m_normalBuffer;
  nvvk::Buffer   m_uvBuffer;
  nvvk::Buffer   m_indexBuffer;
  nvvk::Buffer   m_materialBuffer;
  nvvk::Buffer   m_matrixBuffer;
  nvvk::Buffer   m_rtPrimLookup;

  // Information pushed at each draw call
  struct ObjPushConstant
  {
    nvmath::vec3f lightPosition{0.f, 4.5f, 0.f};
    int           instanceId{0};  // To retrieve the transformation matrix
    float         lightIntensity{10.f};
    int           lightType{0};  // 0: point, 1: infinite
    int           materialId{0};
  };
  ObjPushConstant m_pushConstant;

  // Graphic pipeline
  vk::PipelineLayout          m_pipelineLayout;
  vk::Pipeline                m_graphicsPipeline;
  nvvk::DescriptorSetBindings m_descSetLayoutBind;
  vk::DescriptorPool          m_descPool;
  vk::DescriptorSetLayout     m_descSetLayout;
  vk::DescriptorSet           m_descSet;

  nvvk::Buffer               m_cameraMat;  // Device-Host of the camera matrices
  std::vector<nvvk::Texture> m_textures;   // vector of all textures of the scene

  nvvk::AllocatorDedicated m_alloc;  // Allocator for buffer, images, acceleration structures
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

  // #VKRay
  nvvk::RaytracingBuilderKHR::BlasInput primitiveToGeometry(const nvh::GltfPrimMesh& prim);

  void initRayTracing();
  void createBottomLevelAS();
  void createTopLevelAS();
  void createRtDescriptorSet();
  void updateRtDescriptorSet();
  void createRtPipeline();
  void createRtShaderBindingTable();
  void raytrace(const vk::CommandBuffer& cmdBuf, const nvmath::vec4f& clearColor);
  void updateFrame();
  void resetFrame();

  vk::PhysicalDeviceRayTracingPipelinePropertiesKHR   m_rtProperties;
  nvvk::RaytracingBuilderKHR                          m_rtBuilder;
  nvvk::DescriptorSetBindings                         m_rtDescSetLayoutBind;
  vk::DescriptorPool                                  m_rtDescPool;
  vk::DescriptorSetLayout                             m_rtDescSetLayout;
  vk::DescriptorSet                                   m_rtDescSet;
  std::vector<vk::RayTracingShaderGroupCreateInfoKHR> m_rtShaderGroups;
  vk::PipelineLayout                                  m_rtPipelineLayout;
  vk::Pipeline                                        m_rtPipeline;
  nvvk::Buffer                                        m_rtSBTBuffer;

  struct RtPushConstant
  {
    nvmath::vec4f clearColor;
    nvmath::vec3f lightPosition;
    float         lightIntensity;
    int           lightType;
    int           frame{0};
  } m_rtPushConstants;
>>>>>>> b7ba7d4fdc4128cefb7ab1d6097cc6d21c37d817
};
