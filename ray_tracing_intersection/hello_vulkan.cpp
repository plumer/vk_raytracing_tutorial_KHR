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
#include <random>

#include "fileformats/stb_image.h"
#include "hello_vulkan.h"
#include "nvh/fileoperations.hpp"
//#include "nvvk/commands_vk.hpp"
#include "nvvk/descriptorsets_vk.hpp"
#include "nvvk/pipeline_vk.hpp"
#include "nvvk/renderpasses_vk.hpp"
#include "nvvk/shaders_vk.hpp"
#include "obj_loader.h"
#define GLM_FORCE_RADIANS
#include <glm/gtx/transform.hpp>

#include "logging.h"

// Holding the camera matrices
struct CameraMatrices {
    nvmath::mat4f view;
    nvmath::mat4f proj;
    nvmath::mat4f viewInverse;
    // #VKRay
    nvmath::mat4f projInverse;
};

//--------------------------------------------------------------------------------------------------
// Keep the handle on the device
// Initialize the tool to do all our allocations: buffers, images
//
void HelloVulkan::Setup(const vk::Instance& instance, const vk::Device& device,
                        const vk::PhysicalDevice& physicalDevice, u32 queueFamily)
{
    AppBase::Setup(instance, device, physicalDevice, queueFamily);
    // m_alloc.init(device, physicalDevice);
    allocator_.Setup(device, physicalDevice);
    CHECK(allocator_.ReadyToUse());
    m_debug.setup(device_);
}

//--------------------------------------------------------------------------------------------------
// Called at each frame to update the camera matrix
//
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
}

//--------------------------------------------------------------------------------------------------
// Describing the layout pushed when rendering
//
void HelloVulkan::BuildDescriptorSetLayout()
{
    using vkDS     = vk::DescriptorSetLayoutBinding;
    using vkDT     = vk::DescriptorType;
    using vkSS     = vk::ShaderStageFlagBits;
    uint32_t nbTxt = static_cast<uint32_t>(m_textures.size());
    uint32_t nbObj = static_cast<uint32_t>(m_objModel.size());

    // Camera matrices (binding = 0)
    DS_layout_bindings_.AddBinding(
        vkDS(0, vkDT::eUniformBuffer, 1, vkSS::eVertex | vkSS::eRaygenKHR));
    // Materials (binding = 1)
    DS_layout_bindings_.AddBinding(vkDS(1, vkDT::eStorageBuffer, nbObj + 1,
                                        vkSS::eVertex | vkSS::eFragment | vkSS::eClosestHitKHR));
    // Scene description (binding = 2)
    DS_layout_bindings_.AddBinding(  //
        vkDS(2, vkDT::eStorageBuffer, 1, vkSS::eVertex | vkSS::eFragment | vkSS::eClosestHitKHR));
    // Textures (binding = 3)
    DS_layout_bindings_.AddBinding(
        vkDS(3, vkDT::eCombinedImageSampler, nbTxt, vkSS::eFragment | vkSS::eClosestHitKHR));
    // Materials Index (binding = 4)
    DS_layout_bindings_.AddBinding(
        vkDS(4, vkDT::eStorageBuffer, nbObj + 1, vkSS::eFragment | vkSS::eClosestHitKHR));
    // Storing vertices (binding = 5)
    DS_layout_bindings_.AddBinding(  //
        vkDS(5, vkDT::eStorageBuffer, nbObj, vkSS::eClosestHitKHR));
    // Storing indices (binding = 6)
    DS_layout_bindings_.AddBinding(  //
        vkDS(6, vkDT::eStorageBuffer, nbObj, vkSS::eClosestHitKHR));
    // Storing spheres (binding = 7)
    DS_layout_bindings_.AddBinding(  //
        vkDS(7, vkDT::eStorageBuffer, 1, vkSS::eClosestHitKHR | vkSS::eIntersectionKHR));


    m_descSetLayout = DS_layout_bindings_.MakeLayout(device_);
    m_descPool      = DS_layout_bindings_.MakePool(device_, 1);
    m_descSet       = nvvk::allocateDescriptorSet(device_, m_descPool, m_descSetLayout);
}

//--------------------------------------------------------------------------------------------------
// Setting up the buffers in the descriptor set
//
void HelloVulkan::UpdateDescriptorSet()
{
    std::vector<vk::WriteDescriptorSet> writes;

    // Camera matrices and scene description
    vk::DescriptorBufferInfo dbiUnif{m_cameraMat.handle, 0, VK_WHOLE_SIZE};
    writes.emplace_back(DS_layout_bindings_.MakeWrite(m_descSet, 0, &dbiUnif));
    vk::DescriptorBufferInfo dbiSceneDesc{m_sceneDesc.handle, 0, VK_WHOLE_SIZE};
    writes.emplace_back(DS_layout_bindings_.MakeWrite(m_descSet, 2, &dbiSceneDesc));

    // All material buffers, 1 buffer per OBJ
    std::vector<vk::DescriptorBufferInfo> dbiMat;
    std::vector<vk::DescriptorBufferInfo> dbiMatIdx;
    std::vector<vk::DescriptorBufferInfo> dbiVert;
    std::vector<vk::DescriptorBufferInfo> dbiIdx;
    for (auto& obj : m_objModel) {
        dbiMat.emplace_back(obj.matColorBuffer.handle, 0, VK_WHOLE_SIZE);
        dbiMatIdx.emplace_back(obj.matIndexBuffer.handle, 0, VK_WHOLE_SIZE);
        dbiVert.emplace_back(obj.vertexBuffer.handle, 0, VK_WHOLE_SIZE);
        dbiIdx.emplace_back(obj.indexBuffer.handle, 0, VK_WHOLE_SIZE);
    }
    dbiMat.emplace_back(m_spheresMatColorBuffer.handle, 0, VK_WHOLE_SIZE);
    dbiMatIdx.emplace_back(m_spheresMatIndexBuffer.handle, 0, VK_WHOLE_SIZE);

    writes.emplace_back(DS_layout_bindings_.MakeWriteArray(m_descSet, 1, dbiMat.data()));
    writes.emplace_back(DS_layout_bindings_.MakeWriteArray(m_descSet, 4, dbiMatIdx.data()));
    writes.emplace_back(DS_layout_bindings_.MakeWriteArray(m_descSet, 5, dbiVert.data()));
    writes.emplace_back(DS_layout_bindings_.MakeWriteArray(m_descSet, 6, dbiIdx.data()));

    vk::DescriptorBufferInfo dbiSpheres{m_spheresBuffer.handle, 0, VK_WHOLE_SIZE};
    writes.emplace_back(DS_layout_bindings_.MakeWrite(m_descSet, 7, &dbiSpheres));

    // All texture samplers
    std::vector<vk::DescriptorImageInfo> diit;
    for (auto& texture : m_textures) {
        diit.push_back(texture.descriptor);
    }
    writes.emplace_back(DS_layout_bindings_.MakeWriteArray(m_descSet, 3, diit.data()));

    // Writing the information
    device_.updateDescriptorSets(static_cast<uint32_t>(writes.size()), writes.data(), 0, nullptr);
}

//--------------------------------------------------------------------------------------------------
// Creating the pipeline layout
//
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
    gpb.addShader(nvh::loadFile("shaders/vert_shader.vert.spv", true, paths), vkSS::eVertex);
    gpb.addShader(nvh::loadFile("shaders/frag_shader.frag.spv", true, paths), vkSS::eFragment);
    gpb.addBindingDescription({0, sizeof(VertexObj)});
    gpb.addAttributeDescriptions(
        {{0, 0, vk::Format::eR32G32B32Sfloat, offsetof(VertexObj, pos)},
         {1, 0, vk::Format::eR32G32B32Sfloat, offsetof(VertexObj, nrm)},
         {2, 0, vk::Format::eR32G32B32Sfloat, offsetof(VertexObj, color)},
         {3, 0, vk::Format::eR32G32Sfloat, offsetof(VertexObj, texCoord)}});

    m_graphicsPipeline = gpb.createPipeline();
    m_debug.setObjectName(m_graphicsPipeline, "Graphics");
}

//--------------------------------------------------------------------------------------------------
// Loading the OBJ file and setting up all buffers
//
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

//--------------------------------------------------------------------------------------------------
// Creating the uniform buffer holding the camera matrices
// - Buffer is host visible
//
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
    // use vkpbr code
    vkpbr::CommandPool cmd_pool(device_, graphics_queue_index_);
    auto               cmd_buffer = cmd_pool.MakeCmdBuffer();
    m_sceneDesc = allocator_.MakeBuffer(cmd_buffer, m_objInstance, vkBU::eStorageBuffer);
    cmd_pool.SubmitAndWait(cmd_buffer);
    allocator_.ReleaseAllStagingBuffers();

    m_debug.setObjectName(m_sceneDesc.handle, "sceneDesc");
}

//--------------------------------------------------------------------------------------------------
// Creating all textures and samplers
//
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
        auto                   imageCreateInfo = nvvk::makeImage2DCreateInfo(imgSize, format);

        // Creating the dummy texure
        vkpbr::UniqueMemoryImage image =
            allocator_.MakeImage(cmdBuf, bufferSize, color.data(), imageCreateInfo);

        vk::ImageViewCreateInfo ivInfo =
            nvvk::makeImageViewCreateInfo(image.handle, imageCreateInfo);
        texture = allocator_.MakeTexture(image, ivInfo, samplerCreateInfo);
        CHECK(texture.descriptor.sampler);

        // The image format must be in VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL
        nvvk::cmdBarrierImageLayout(cmdBuf, texture.handle, vk::ImageLayout::eUndefined,
                                    vk::ImageLayout::eShaderReadOnlyOptimal);
        m_textures.push_back(texture);
    } else {
        // Uploading all images
        for (const auto& texture : textures) {
            std::stringstream o;
            int               texWidth, texHeight, texChannels;
            o << "media/textures/" << texture;
            std::string txtFile = nvh::findFile(o.str(), defaultSearchPaths);

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
            auto imgSize = vk::Extent2D(texWidth, texHeight);
            auto imageCreateInfo =
                nvvk::makeImage2DCreateInfo(imgSize, format, vkIU::eSampled, true);

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
}

//--------------------------------------------------------------------------------------------------
// Destroying all allocations
//
void HelloVulkan::destroyResources()
{
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
    cmdBuf.setViewport(
        0, {vk::Viewport(0, 0, (float)window_size_.width, (float)window_size_.height, 0, 1)});
    cmdBuf.setScissor(0, {{{0, 0}, {window_size_.width, window_size_.height}}});

    // Drawing all triangles
    cmdBuf.bindPipeline(vkPBP::eGraphics, m_graphicsPipeline);
    cmdBuf.bindDescriptorSets(vkPBP::eGraphics, m_pipelineLayout, 0, {m_descSet}, {});
    for (int i = 0; i < m_objInstance.size(); ++i) {
        auto& inst                = m_objInstance[i];
        auto& model               = m_objModel[inst.objIndex];
        m_pushConstant.instanceId = i;  // Telling which instance is drawn
        cmdBuf.pushConstants<ObjPushConstant>(m_pipelineLayout, vkSS::eVertex | vkSS::eFragment, 0,
                                              m_pushConstant);

        cmdBuf.bindVertexBuffers(0, {model.vertexBuffer.handle}, {offset});
        cmdBuf.bindIndexBuffer(model.indexBuffer.handle, 0, vk::IndexType::eUint32);
        cmdBuf.drawIndexed(model.nbIndices, 1, 0, 0, 0);
    }
    m_debug.endLabel(cmdBuf);
}

//--------------------------------------------------------------------------------------------------
// Handling resize of the window
//
void HelloVulkan::WindowResizeCallback(int w, int h)
{
    AppBase::WindowResizeCallback(w, h);
    createOffscreenRender();
    updatePostDescriptorSet();
    updateRtDescriptorSet();
}

//////////////////////////////////////////////////////////////////////////
// Post-processing
//////////////////////////////////////////////////////////////////////////

//--------------------------------------------------------------------------------------------------
// Creating an offscreen frame buffer and the associated render pass
//
void HelloVulkan::createOffscreenRender()
{
    // Creating the color image
    {
        auto colorCreateInfo = nvvk::makeImage2DCreateInfo(window_size_, m_offscreenColorFormat,
                                                           vk::ImageUsageFlagBits::eColorAttachment
                                                               | vk::ImageUsageFlagBits::eSampled
                                                               | vk::ImageUsageFlagBits::eStorage);

        vkpbr::UniqueMemoryImage image = allocator_.MakeImage(colorCreateInfo);
        vk::ImageViewCreateInfo  ivInfo =
            nvvk::makeImageViewCreateInfo(image.handle, colorCreateInfo);
        m_offscreenColor = allocator_.MakeTexture(image, ivInfo, vk::SamplerCreateInfo());
        m_offscreenColor.descriptor.imageLayout = vk::ImageLayout::eGeneral;
    }

    // Creating the depth buffer
    auto depthCreateInfo =
        nvvk::makeImage2DCreateInfo(window_size_, m_offscreenDepthFormat,
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
        nvvk::cmdBarrierImageLayout(cmdBuf, m_offscreenColor.handle, vk::ImageLayout::eUndefined,
                                    vk::ImageLayout::eGeneral);
        nvvk::cmdBarrierImageLayout(cmdBuf, m_offscreenDepth.handle, vk::ImageLayout::eUndefined,
                                    vk::ImageLayout::eDepthStencilAttachmentOptimal,
                                    vk::ImageAspectFlagBits::eDepth);

        genCmdBuf.SubmitAndWait(cmdBuf);
    }

    // Creating a renderpass for the offscreen
    if (!m_offscreenRenderPass) {
        m_offscreenRenderPass =
            nvvk::createRenderPass(device_, {m_offscreenColorFormat}, m_offscreenDepthFormat, 1,
                                   true, true, vk::ImageLayout::eGeneral,
                                   vk::ImageLayout::eGeneral);
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
}

//--------------------------------------------------------------------------------------------------
// The pipeline is how things are rendered, which shaders, type of primitives, depth test and more
//
void HelloVulkan::createPostPipeline()
{
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

    post_DS_layout_bindings_.AddBinding(vkDS(0, vkDT::eCombinedImageSampler, 1, vkSS::eFragment));
    m_postDescSetLayout = post_DS_layout_bindings_.MakeLayout(device_);
    m_postDescPool      = post_DS_layout_bindings_.MakePool(device_);
    m_postDescSet       = nvvk::allocateDescriptorSet(device_, m_postDescPool, m_postDescSetLayout);
}

//--------------------------------------------------------------------------------------------------
// Update the output
//
void HelloVulkan::updatePostDescriptorSet()
{
    vk::WriteDescriptorSet writeDescriptorSets = post_DS_layout_bindings_.MakeWrite(
        m_postDescSet, 0,
        reinterpret_cast<const vk::DescriptorImageInfo*>(&m_offscreenColor.descriptor));
    device_.updateDescriptorSets(writeDescriptorSets, nullptr);
}

//--------------------------------------------------------------------------------------------------
// Draw a full screen quad with the attached image
//
void HelloVulkan::drawPost(vk::CommandBuffer cmdBuf)
{
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
}

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

//--------------------------------------------------------------------------------------------------
// Initialize Vulkan ray tracing
// #VKRay
void HelloVulkan::initRayTracing()
{
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
    all_blas.reserve(m_objModel.size());
    for (const auto& obj : m_objModel) {
        auto blas = objectToVkGeometryKHR(obj);
        all_blas.emplace_back(blas);
    }
    {
        auto blas = sphereToVkGeometryKHR();
        all_blas.emplace_back(blas);
    }
    m_rtBuilder.buildBlas(all_blas, vk::BuildAccelerationStructureFlagBitsKHR::ePreferFastTrace);
}

void HelloVulkan::createTopLevelAS()
{

    std::vector<vkpbr::RaytracingBuilderKHR::Instance> tlas_instances;
    tlas_instances.reserve(m_objInstance.size());
    for (int i = 0; i < cast_i32(m_objInstance.size()); ++i) {
        vkpbr::RaytracingBuilderKHR::Instance instance;
        memcpy(&instance.transform, &m_objInstance[i].transform, sizeof instance.transform);
        instance.instanceId = i;
        instance.blasId     = m_objInstance[i].objIndex;
        instance.hitGroupId = 0;
        instance.flags      = vk::GeometryInstanceFlagBitsKHR::eTriangleFacingCullDisable;
        tlas_instances.push_back(instance);
    }
    {
        vkpbr::RaytracingBuilderKHR::Instance sphere_instance;
        memcpy(&sphere_instance.transform, &m_objInstance[0].transform,
               sizeof sphere_instance.transform);
        sphere_instance.instanceId = cast_u32(tlas_instances.size());
        sphere_instance.blasId     = cast_u32(m_objModel.size());
        sphere_instance.hitGroupId = 1;
        sphere_instance.flags      = vk::GeometryInstanceFlagBitsKHR::eTriangleFacingCullDisable;
        tlas_instances.emplace_back(sphere_instance);
    }
    m_rtBuilder.buildTlas(tlas_instances,
                          vk::BuildAccelerationStructureFlagBitsKHR::ePreferFastTrace);
}

//--------------------------------------------------------------------------------------------------
// This descriptor set holds the Acceleration structure and the output image
//
void HelloVulkan::createRtDescriptorSet()
{
    using vkDT   = vk::DescriptorType;
    using vkSS   = vk::ShaderStageFlagBits;
    using vkDSLB = vk::DescriptorSetLayoutBinding;

    rt_DS_layout_bindings_.AddBinding(vkDSLB(0, vkDT::eAccelerationStructureKHR, 1,
                                             vkSS::eRaygenKHR | vkSS::eClosestHitKHR));  // TLAS
    rt_DS_layout_bindings_.AddBinding(
        vkDSLB(1, vkDT::eStorageImage, 1, vkSS::eRaygenKHR));  // Output image

    m_rtDescPool      = rt_DS_layout_bindings_.MakePool(device_);
    m_rtDescSetLayout = rt_DS_layout_bindings_.MakeLayout(device_);
    m_rtDescSet       = device_.allocateDescriptorSets({m_rtDescPool, 1, &m_rtDescSetLayout})[0];

    vk::AccelerationStructureKHR                   tlas = m_rtBuilder.getAccelerationStructure();
    vk::WriteDescriptorSetAccelerationStructureKHR descASInfo;
    descASInfo.setAccelerationStructureCount(1);
    descASInfo.setPAccelerationStructures(&tlas);
    vk::DescriptorImageInfo imageInfo{
        {}, m_offscreenColor.descriptor.imageView, vk::ImageLayout::eGeneral};

    std::vector<vk::WriteDescriptorSet> writes;
    writes.emplace_back(rt_DS_layout_bindings_.MakeWrite(m_rtDescSet, 0, &descASInfo));
    writes.emplace_back(rt_DS_layout_bindings_.MakeWrite(m_rtDescSet, 1, &imageInfo));
    device_.updateDescriptorSets(static_cast<uint32_t>(writes.size()), writes.data(), 0, nullptr);
}


//--------------------------------------------------------------------------------------------------
// Writes the output image to the descriptor set
// - Required when changing resolution
//
void HelloVulkan::updateRtDescriptorSet()
{
    using vkDT = vk::DescriptorType;

    // (1) Output buffer
    vk::DescriptorImageInfo imageInfo{
        {}, m_offscreenColor.descriptor.imageView, vk::ImageLayout::eGeneral};
    vk::WriteDescriptorSet wds{m_rtDescSet, 1, 0, 1, vkDT::eStorageImage, &imageInfo};
    device_.updateDescriptorSets(wds, nullptr);
}


//--------------------------------------------------------------------------------------------------
// Pipeline for the ray tracer: all shaders, raygen, chit, miss
//
void HelloVulkan::createRtPipeline()
{
    std::vector<std::string> paths = defaultSearchPaths;

    vk::ShaderModule raygenSM =
        nvvk::createShaderModule(device_,  //
                                 nvh::loadFile("shaders/raytrace.rgen.spv", true, paths));
    vk::ShaderModule missSM =
        nvvk::createShaderModule(device_,  //
                                 nvh::loadFile("shaders/raytrace.rmiss.spv", true, paths));

    // The second miss shader is invoked when a shadow ray misses the geometry. It
    // simply indicates that no occlusion has been found
    vk::ShaderModule shadowmissSM =
        nvvk::createShaderModule(device_,
                                 nvh::loadFile("shaders/raytraceShadow.rmiss.spv", true, paths));


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

    // Hit Group0 - Closest Hit
    vk::ShaderModule chitSM =
        nvvk::createShaderModule(device_,  //
                                 nvh::loadFile("shaders/raytrace.rchit.spv", true, paths));

    {
        vk::RayTracingShaderGroupCreateInfoKHR hg{
            vk::RayTracingShaderGroupTypeKHR::eTrianglesHitGroup, VK_SHADER_UNUSED_KHR,
            VK_SHADER_UNUSED_KHR, VK_SHADER_UNUSED_KHR, VK_SHADER_UNUSED_KHR};
        stages.push_back({{}, vk::ShaderStageFlagBits::eClosestHitKHR, chitSM, "main"});
        hg.setClosestHitShader(static_cast<uint32_t>(stages.size() - 1));
        m_rtShaderGroups.push_back(hg);
    }

    // Hit Group1 - Closest Hit + Intersection (procedural)
    vk::ShaderModule chit2SM =
        nvvk::createShaderModule(device_,  //
                                 nvh::loadFile("shaders/raytrace2.rchit.spv", true, paths));
    vk::ShaderModule rintSM =
        nvvk::createShaderModule(device_,  //
                                 nvh::loadFile("shaders/raytrace.rint.spv", true, paths));
    {
        vk::RayTracingShaderGroupCreateInfoKHR hg{
            vk::RayTracingShaderGroupTypeKHR::eProceduralHitGroup, VK_SHADER_UNUSED_KHR,
            VK_SHADER_UNUSED_KHR, VK_SHADER_UNUSED_KHR, VK_SHADER_UNUSED_KHR};
        stages.push_back({{}, vk::ShaderStageFlagBits::eClosestHitKHR, chit2SM, "main"});
        hg.setClosestHitShader(static_cast<uint32_t>(stages.size() - 1));
        stages.push_back({{}, vk::ShaderStageFlagBits::eIntersectionKHR, rintSM, "main"});
        hg.setIntersectionShader(static_cast<uint32_t>(stages.size() - 1));
        m_rtShaderGroups.push_back(hg);
    }

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

    rayPipelineInfo.setGroupCount(static_cast<uint32_t>(
        m_rtShaderGroups.size()));  // 1-raygen, n-miss, n-(hit[+anyhit+intersect])
    rayPipelineInfo.setPGroups(m_rtShaderGroups.data());

    rayPipelineInfo.setMaxRecursionDepth(2);  // Ray depth
    rayPipelineInfo.setLayout(m_rtPipelineLayout);
    m_rtPipeline = device_.createRayTracingPipelineKHR({}, rayPipelineInfo).value;

    device_.destroy(raygenSM);
    device_.destroy(missSM);
    device_.destroy(shadowmissSM);
    device_.destroy(chitSM);
    device_.destroy(chit2SM);
    device_.destroy(rintSM);
}

//--------------------------------------------------------------------------------------------------
// The Shader Binding Table (SBT)
// - getting all shader handles and writing them in a SBT buffer
// - Besides exception, this could be always done like this
//   See how the SBT buffer is used in run()
//
void HelloVulkan::createRtShaderBindingTable()
{
    auto groupCount = static_cast<uint32_t>(m_rtShaderGroups.size());  // 3 shaders: raygen, miss,
                                                                       // chit
    uint32_t groupHandleSize = m_rtProperties.shaderGroupHandleSize;   // Size of a program
                                                                       // identifier

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
}

//--------------------------------------------------------------------------------------------------
// Ray Tracing the scene
//
void HelloVulkan::raytrace(const vk::CommandBuffer& cmdBuf, const glm::vec4& clearColor)
{
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
