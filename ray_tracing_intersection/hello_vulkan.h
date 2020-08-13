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

#include "types.h"

#define NVVK_ALLOC_DEDICATED
#include "nvvk/allocator_vk.hpp"
#include "vk_appbase.h"
#include "vk_memory.h"
#include "vk_utils.h"
// #VKRay
#include "nvvk/raytraceKHR_vk.hpp"

//--------------------------------------------------------------------------------------------------
// Simple rasterizer of OBJ objects
// - Each OBJ loaded are stored in an `ObjModel` and referenced by a `ObjInstance`
// - It is possible to have many `ObjInstance` referencing the same `ObjModel`
// - Rendering is done in an offscreen framebuffer
// - The image of the framebuffer is displayed in post-process in a full-screen quad
//
class HelloVulkan : public vkpbr::AppBase
{
  public:
    HelloVulkan() = default;
    void Setup(const vk::Instance& instance, const vk::Device& device,
               const vk::PhysicalDevice& physicalDevice, u32 queueFamily) override;
    void BuildDescriptorSetLayout();
    void BuildGraphicsPipeline();
    void LoadModel(const std::string& filename, nvmath::mat4f transform = nvmath::mat4f(1));
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
        uint32_t      objIndex{0};     // Reference to the `m_objModel`
        uint32_t      txtOffset{0};    // Offset in `m_textures`
        nvmath::mat4f transform{1};    // Position of the instance
        nvmath::mat4f transformIT{1};  // Inverse transpose
        nvmath::mat4f dummy;
    };

    // Information pushed at each draw call
    struct ObjPushConstant {
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
    vk::PipelineLayout           m_pipelineLayout;
    vk::Pipeline                 m_graphicsPipeline;
    vkpbr::DescriptorSetBindings DS_layout_bindings_;
    vk::DescriptorPool           m_descPool;
    vk::DescriptorSetLayout      m_descSetLayout;
    vk::DescriptorSet            m_descSet;

    vkpbr::UniqueMemoryBuffer  m_cameraMat;  // Device-Host of the camera matrices
    vkpbr::UniqueMemoryBuffer  m_sceneDesc;  // Device buffer of the OBJ instances
    std::vector<nvvk::Texture> m_textures;   // vector of all textures of the scene

    nvvk::AllocatorDedicated m_alloc;  // Allocator for buffer, images, acceleration structures
    vkpbr::UniqueMemoryAllocator allocator_;
    nvvk::DebugUtil          m_debug;  // Utility to name objects

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
    nvvk::Texture                m_offscreenColor;
    vk::Format                   m_offscreenColorFormat{vk::Format::eR32G32B32A32Sfloat};
    nvvk::Texture                m_offscreenDepth;
    vk::Format                   m_offscreenDepthFormat{vk::Format::eD32Sfloat};

    // #VKRay
    void                             initRayTracing();
    nvvk::RaytracingBuilderKHR::Blas objectToVkGeometryKHR(const ObjModel& model);
    void                             createBottomLevelAS();
    void                             createTopLevelAS();
    void                             createRtDescriptorSet();
    void                             updateRtDescriptorSet();
    void                             createRtPipeline();
    void                             createRtShaderBindingTable();
    void raytrace(const vk::CommandBuffer& cmdBuf, const nvmath::vec4f& clearColor);


    vk::PhysicalDeviceRayTracingPropertiesKHR           m_rtProperties;
    nvvk::RaytracingBuilderKHR                          m_rtBuilder;
    vkpbr::DescriptorSetBindings                        rt_DS_layout_bindings_;
    vk::DescriptorPool                                  m_rtDescPool;
    vk::DescriptorSetLayout                             m_rtDescSetLayout;
    vk::DescriptorSet                                   m_rtDescSet;
    std::vector<vk::RayTracingShaderGroupCreateInfoKHR> m_rtShaderGroups;
    vk::PipelineLayout                                  m_rtPipelineLayout;
    vk::Pipeline                                        m_rtPipeline;
    vkpbr::UniqueMemoryBuffer                           m_rtSBTBuffer;

    struct RtPushConstant {
        nvmath::vec4f clearColor;
        nvmath::vec3f lightPosition;
        float         lightIntensity;
        int           lightType;
    } m_rtPushConstants;


    struct Sphere {
        nvmath::vec3f center;
        float         radius;
    };

    struct Aabb {
        nvmath::vec3f minimum;
        nvmath::vec3f maximum;
    };

    nvvk::RaytracingBuilderKHR::Blas sphereToVkGeometryKHR();

    std::vector<Sphere>       m_spheres;                // All spheres
    vkpbr::UniqueMemoryBuffer m_spheresBuffer;          // Buffer holding the spheres
    vkpbr::UniqueMemoryBuffer m_spheresAabbBuffer;      // Buffer of all Aabb
    vkpbr::UniqueMemoryBuffer m_spheresMatColorBuffer;  // Multiple materials
    vkpbr::UniqueMemoryBuffer m_spheresMatIndexBuffer;  // Define which sphere uses which material
    void                      createSpheres();
};
