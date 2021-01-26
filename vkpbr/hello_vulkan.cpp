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
#include "io.h"
#include "logging.h"
#include "meshloader.h"
#include "nvvk/pipeline_vk.hpp"
#include "vk_utils.h"

#include <glm/gtx/transform.hpp>


// Holding the camera matrices
struct CameraMatrices {
    glm::mat4 view;
    glm::mat4 proj;
    glm::mat4 viewInverse;
    // #VKRay
    glm::mat4 projInverse;
};


//--------------------------------------------------------------------------------------------------
// Keep the handle on the device
// Initialize the tool to do all our allocations: buffers, images
//
void HelloVulkan::setup(const vk::Instance& instance, const vk::Device& device,
                        const vk::PhysicalDevice& physicalDevice, uint32_t queueFamily)
{
    AppBase::setup(instance, device, physicalDevice, queueFamily);
    // m_alloc.init(device, physicalDevice);
    allocator_.Setup(device, physicalDevice);
    m_debug.setup(m_device);
}

void HelloVulkan::onKeyboard(int key, int scancode, int action, int mods)
{
    if (key == GLFW_KEY_F4) {
        auto view = camera_->ViewMatrix();
        view      = glm::transpose(view);

        LOG(INFO) << " view matrix = ";
        for (int i = 0; i < 4; ++i)
            printf("[%7.6g, %7.6f, %7.6f, %7.6f]\n", view[i][0], view[i][1], view[i][2],
                   view[i][3]);
    }
    vkpbr::AppBase::onKeyboard(key, scancode, action, mods);
}

//--------------------------------------------------------------------------------------------------
// Called at each frame to update the camera matrix
//
void HelloVulkan::updateUniformBuffer(const vk::CommandBuffer& cmdBuf)
{
    const float aspectRatio = m_size.width / static_cast<float>(m_size.height);

    CameraMatrices ubo = {};
#ifdef USE_NV_CAMERA
    nvmath::mat4f nv_view_mat = CameraManip.getMatrix();
    static_assert(sizeof(nv_view_mat) == sizeof(ubo.view), "glm::mat4");
    memcpy(&ubo.view, &nv_view_mat, sizeof(nv_view_mat));
    ubo.proj = glm::perspective(glm::radians(CameraManip.getFov()), aspectRatio, 0.1f, 1000.0f);
#else
    ubo.view = camera_->ViewMatrix();
    ubo.proj = glm::perspective(glm::radians(camera_->Fov()), aspectRatio, 0.1f, 2000.f);
#endif
    ubo.proj[1][1] *= -1;  // Inverting Y for Vulkan
    ubo.viewInverse = glm::inverse(ubo.view);
    // #VKRay
    ubo.projInverse = glm::inverse(ubo.proj);

    cmdBuf.updateBuffer<CameraMatrices>(m_cameraMat.handle, 0, ubo);

    // Making sure the matrix buffer will be available
    vk::MemoryBarrier mb{vk::AccessFlagBits::eTransferWrite, vk::AccessFlagBits::eShaderRead};
    cmdBuf.pipelineBarrier(vk::PipelineStageFlagBits::eTransfer,
                           vk::PipelineStageFlagBits::eVertexShader
                               | vk::PipelineStageFlagBits::eAccelerationStructureBuildKHR,
                           vk::DependencyFlagBits::eDeviceGroup, {mb}, {}, {});
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
    m_descSetLayoutBind.AddBinding(
        vkDS(0, vkDT::eUniformBuffer, 1, vkSS::eVertex | vkSS::eRaygenKHR));
    // Scene description (binding = 2)
    m_descSetLayoutBind.AddBinding(  //
        vkDS(2, vkDT::eStorageBuffer, 1, vkSS::eVertex | vkSS::eFragment | vkSS::eClosestHitKHR));
    // Textures (binding = 3)
    m_descSetLayoutBind.AddBinding(
        vkDS(3, vkDT::eCombinedImageSampler, nbTxt, vkSS::eFragment | vkSS::eClosestHitKHR));
    // Storing vertices (binding = 5)
    m_descSetLayoutBind.AddBinding(  //
        vkDS(5, vkDT::eStorageBuffer, nbObj, vkSS::eClosestHitKHR));
    // Storing indices (binding = 6)
    m_descSetLayoutBind.AddBinding(  //
        vkDS(6, vkDT::eStorageBuffer, nbObj, vkSS::eClosestHitKHR));

    // Uniform materials (not per-object).
    m_descSetLayoutBind.AddBinding(                  // 1 buffer of array.
        vkDS(RtDsb::kMaterials, vkDT::eStorageBuffer, 1, vkSS::eFragment | vkSS::eClosestHitKHR));


    m_descSetLayout = m_descSetLayoutBind.MakeLayout(m_device);
    m_descPool      = m_descSetLayoutBind.MakePool(m_device, 1);

    auto ds_alloc_info = vk::DescriptorSetAllocateInfo()
                             .setDescriptorPool(m_descPool)
                             .setSetLayouts(m_descSetLayout);

    m_descSet = m_device.allocateDescriptorSets(ds_alloc_info).front();
}

//--------------------------------------------------------------------------------------------------
// Setting up the buffers in the descriptor set
//
void HelloVulkan::updateDescriptorSet()
{
    std::vector<vk::WriteDescriptorSet> writes;

    // Camera matrices and scene description
    vk::DescriptorBufferInfo dbiUnif{m_cameraMat.handle, 0, VK_WHOLE_SIZE};
    writes.emplace_back(m_descSetLayoutBind.MakeWrite(m_descSet, 0, &dbiUnif));
    vk::DescriptorBufferInfo dbiSceneDesc{m_sceneDesc.handle, 0, VK_WHOLE_SIZE};
    writes.emplace_back(m_descSetLayoutBind.MakeWrite(m_descSet, 2, &dbiSceneDesc));

    // All material buffers, 1 buffer per OBJ
    std::vector<vk::DescriptorBufferInfo> dbiMat;
    std::vector<vk::DescriptorBufferInfo> dbiMatIdx;
    std::vector<vk::DescriptorBufferInfo> dbiVert;
    std::vector<vk::DescriptorBufferInfo> dbiIdx;
    for (auto& obj : m_objModel) {
        dbiVert.emplace_back(obj.vertexBuffer.handle, 0, VK_WHOLE_SIZE);
        dbiIdx.emplace_back(obj.indexBuffer.handle, 0, VK_WHOLE_SIZE);
    }
    writes.emplace_back(m_descSetLayoutBind.MakeWriteArray(m_descSet, 5, dbiVert.data()));
    writes.emplace_back(m_descSetLayoutBind.MakeWriteArray(m_descSet, 6, dbiIdx.data()));

    // All texture samplers
    std::vector<vk::DescriptorImageInfo> diit;
    for (auto& texture : m_textures) {
        diit.emplace_back(texture.descriptor);
    }
    writes.emplace_back(m_descSetLayoutBind.MakeWriteArray(m_descSet, 3, diit.data()));

    auto dbi_universal_mtls = vk::DescriptorBufferInfo()
                                  .setBuffer(materials_buffer_.handle)
                                  .setOffset(0)
                                  .setRange(VK_WHOLE_SIZE);
    writes.emplace_back(
        m_descSetLayoutBind.MakeWrite(m_descSet, RtDsb::kMaterials, &dbi_universal_mtls));

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
    pipelineLayoutCreateInfo.setSetLayouts(descSetLayout);
    pipelineLayoutCreateInfo.setPushConstantRanges(pushConstantRanges);
    m_pipelineLayout = m_device.createPipelineLayout(pipelineLayoutCreateInfo);

    // Creating the Pipeline
    std::vector<std::string>                paths = defaultSearchPaths;
    nvvk::GraphicsPipelineGeneratorCombined gpb(m_device, m_pipelineLayout, m_offscreenRenderPass);
    gpb.depthStencilState.depthTestEnable = true;
    gpb.addShader(io::LoadBinaryFile("shaders/vert_shader.vert.spv", paths), vkSS::eVertex);
    gpb.addShader(io::LoadBinaryFile("shaders/frag_shader.frag.spv", paths), vkSS::eFragment);

    std::vector<vk::VertexInputAttributeDescription> vertex_attribute_332 = {
        // location, binding, format,        offset
        {0, 0, vk::Format::eR32G32B32Sfloat, offsetof(vkpbr::VertexData, position)},
        {1, 0, vk::Format::eR32G32B32Sfloat, offsetof(vkpbr::VertexData, normal)},
        {2, 0, vk::Format::eR32G32Sfloat, offsetof(vkpbr::VertexData, texture_uv)},
    };
    gpb.addAttributeDescriptions(vertex_attribute_332);
    gpb.addBindingDescription({/* binding_index = */0, /* stride = */sizeof(vkpbr::VertexData)});

    m_graphicsPipeline = gpb.createPipeline();
    m_debug.setObjectName(m_graphicsPipeline, "Graphics");
}

//--------------------------------------------------------------------------------------------------
// Loading the OBJ file and setting up all buffers
//
void HelloVulkan::loadModel(const std::string& filename, glm::mat4 transform)
{
    using vkBU   = vk::BufferUsageFlagBits;
    using vkrtBU = vkrt::BufferUsageFlagBits;

    LOG(INFO) << vkpbr::Format("Loading File:  %s", filename.c_str());
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
    model.num_indices  = static_cast<uint32_t>(loader.m_indices.size());
    model.num_vertices = static_cast<uint32_t>(loader.m_vertices.size());

    // Create the buffers on Device and copy vertices, indices and materials
    vkpbr::CommandPool cmdBufGet(m_device, m_graphicsQueueIndex);
    vk::CommandBuffer  cmdBuf = cmdBufGet.MakeCmdBuffer();
    model.vertexBuffer =
        allocator_.MakeBuffer(cmdBuf, loader.m_vertices,
                              vkBU::eVertexBuffer | vkBU::eStorageBuffer
                                  | vkBU::eShaderDeviceAddress | vkrtBU::eBvhBuildInputReadOnly);
    model.indexBuffer =
        allocator_.MakeBuffer(cmdBuf, loader.m_indices,
                              vkBU::eIndexBuffer | vkBU::eStorageBuffer | vkBU::eShaderDeviceAddress
                                  | vkBU::eAccelerationStructureBuildInputReadOnlyKHR);
    // Creates all textures found
    createTextureImages(cmdBuf, loader.m_textures);
    cmdBufGet.SubmitAndWait(cmdBuf);
    allocator_.ReleaseAllStagingBuffers();

    u32 max_index = *std::max_element(loader.m_matIndx.begin(), loader.m_matIndx.end());
    CHECK_LT(max_index, loader.m_materials.size());

    std::string objNb = std::to_string(instance.objIndex);
    m_debug.setObjectName(model.vertexBuffer.handle, (std::string("vertex_" + objNb).c_str()));
    m_debug.setObjectName(model.indexBuffer.handle, (std::string("index_" + objNb).c_str()));

    m_objModel.emplace_back(model);
    m_objInstance.emplace_back(instance);
}

void HelloVulkan::PrepareScene()
{
    const std::string prefix = "scenes/caustic-glass/";
    // auto glass_ply = LoadPly(io::FindFile(prefix + "geometry/mesh_00001.ply",
    // defaultSearchPaths),
    ///*enforce_triangle=*/true);
    auto glass_ply = LoadObj(io::FindFile("media/scenes/cube.obj", defaultSearchPaths), true);
    {
        for (auto& p : glass_ply.positions) {
            p *= glm::vec3(0.5, 2, 5);
        }
    }
    if (glass_ply.normals.empty())
        glass_ply.normals = ComputeNormals(glass_ply.positions, glass_ply.indices);

    auto plane_ply = LoadPly(io::FindFile(prefix + "geometry/mesh_00002.ply", defaultSearchPaths),
                             /*enforce_triangle=*/true);
    {  // Centers the glass ply to the origin, and places the plane right under it.
        glm::vec3 glass_min{INFINITY}, glass_max{-INFINITY};
        for (const glm::vec3& p : glass_ply.positions) {
            glass_min = glm::min(p, glass_min);
            glass_max = glm::max(p, glass_max);
        }
        auto bbox_mid = (glass_min + glass_max) / 2.0f;

        for (auto& p : glass_ply.positions) {
            p -= bbox_mid;
        }
        for (auto& p : plane_ply.positions) {
            p.y = glass_min.y;
        }
    }

    std::vector<VertexObj> glass_vertices(glass_ply.positions.size());
    auto Glm2NvV3 = [](const glm::vec3& v) { return nvmath::vec3f(v.x, v.y, v.z); };
    auto Glm2NvV2 = [](const glm::vec2& v) { return nvmath::vec2f(v.x, v.y); };
    for (size_t i = 0; i < glass_ply.positions.size(); ++i) {
        glass_vertices[i].pos = Glm2NvV3(glass_ply.positions[i]);
        glass_vertices[i].nrm = Glm2NvV3(glass_ply.normals[i]);
        if (!glass_ply.texture_uvs.empty())
            glass_vertices[i].texCoord = Glm2NvV2(glass_ply.texture_uvs[i]);
    }
    std::vector<VertexObj> plane_vertices(plane_ply.positions.size());
    for (size_t i = 0; i < plane_ply.positions.size(); ++i) {
        plane_vertices[i].pos = Glm2NvV3(plane_ply.positions[i]);
        plane_vertices[i].nrm = Glm2NvV3(plane_ply.normals[i]);
        if (!plane_ply.texture_uvs.empty())
            plane_vertices[i].texCoord = Glm2NvV2(plane_ply.texture_uvs[i]);
    }

    vkpbr::CommandPool cmd_pool(m_device, m_graphicsQueueIndex);
    vk::CommandBuffer  cmd_buffer = cmd_pool.MakeCmdBuffer();

    using vkBU   = vk::BufferUsageFlagBits;
    using vkrtBU = vkrt::BufferUsageFlagBits;
    ObjModel glass_model;
    glass_model.num_vertices = cast_u32(glass_vertices.size());
    glass_model.num_indices  = cast_u32(glass_ply.indices.size());
    glass_model.vertexBuffer =
        allocator_.MakeBuffer(cmd_buffer, glass_vertices,
                              vkBU::eVertexBuffer | vkBU::eStorageBuffer
                                  | vkBU::eShaderDeviceAddress | vkrtBU::eBvhBuildInputReadOnly);
    glass_model.indexBuffer =
        allocator_.MakeBuffer(cmd_buffer, glass_ply.indices,
                              vkBU::eIndexBuffer | vkBU::eStorageBuffer | vkBU::eShaderDeviceAddress
                                  | vkrtBU::eBvhBuildInputReadOnly);

    MaterialObj glass_mtl;
    glass_mtl.ior           = 1.01f;
    glass_mtl.transmittance = {1.0f, 1.0f, 1.0f};
    glass_mtl.emission      = {0.0f, 0.0f, 0.0f};
    glass_mtl.textureID     = -1;

    std::vector<MaterialObj> mtl_wrapper = {glass_mtl};
    std::vector<int> mtl_indices = {0};

    MaterialObj plane_mtl;
    plane_mtl.specular  = {0.2f, 0.2f, 0.2f};
    plane_mtl.diffuse   = {0.64f, 0.64f, 0.64f};
    plane_mtl.specular  = {0.1f, 0.1f, 0.1f};
    plane_mtl.emission  = {0.0f, 0.0f, 0.0f};
    plane_mtl.textureID = -1;
    mtl_wrapper         = {plane_mtl};
    ObjModel plane_model;
    plane_model.num_vertices = cast_u32(plane_vertices.size());
    plane_model.num_indices  = cast_u32(plane_ply.indices.size());
    plane_model.vertexBuffer =
        allocator_.MakeBuffer(cmd_buffer, plane_vertices,
                              vkBU::eVertexBuffer | vkBU::eStorageBuffer
                                  | vkBU::eShaderDeviceAddress | vkrtBU::eBvhBuildInputReadOnly);
    plane_model.indexBuffer =
        allocator_.MakeBuffer(cmd_buffer, plane_ply.indices,
                              vkBU::eIndexBuffer | vkBU::eStorageBuffer | vkBU::eShaderDeviceAddress
                                  | vkrtBU::eBvhBuildInputReadOnly);

    m_objModel.push_back(glass_model);
    m_objModel.push_back(plane_model);

    // Prepares the instances.
    ObjInstance glass_instance;
    glass_instance.objIndex = cast_u32(m_objModel.size() - 2);

    ObjInstance plane_instance;
    plane_instance.objIndex = cast_u32(m_objModel.size() - 1);
    m_objInstance.push_back(glass_instance);
    m_objInstance.push_back(plane_instance);

    createTextureImages(cmd_buffer, {});

    cmd_pool.SubmitAndWait(cmd_buffer);
    allocator_.ReleaseAllStagingBuffers();
}

void HelloVulkan::PrepareCornellBox()
{
    m_pushConstant.lightPosition = {350, 550, 230};
    std::vector<glm::vec3> positions;
    MaterialObj            mtl;

    using vkBU   = vk::BufferUsageFlagBits;
    using vkrtBU = vkrt::BufferUsageFlagBits;

    auto AddMesh = [this](const std::vector<glm::vec3>& positions, const std::vector<int>& indices,
                          const MaterialObj &mtl, int mtl_index, vk::CommandBuffer cmd_buffer) {
        ObjModel                 model;
        std::vector<MaterialObj> mtl_wrapper = {mtl};
        std::vector<int>         mtl_indices = {mtl_index};

        // In the Cornell Box scene, all triangles are in the same plane, thus ComputeNormals()
        // will compute the correct normal vectors.
        std::vector<glm::vec3> normals = ComputeNormals(positions, indices);
        auto                   vertex_data = Interleave(positions, normals);

        model.num_vertices   = cast_u32(positions.size());
        model.num_indices    = cast_u32(indices.size());
        model.vertexBuffer   = allocator_.MakeBuffer(cmd_buffer, vertex_data,
                                                   vkBU::eVertexBuffer | vkBU::eStorageBuffer
                                                       | vkBU::eShaderDeviceAddress
                                                       | vkrtBU::eBvhBuildInputReadOnly);
        model.indexBuffer    = allocator_.MakeBuffer(cmd_buffer, indices,
                                                  vkBU::eIndexBuffer | vkBU::eStorageBuffer
                                                      | vkBU::eShaderDeviceAddress
                                                      | vkrtBU::eBvhBuildInputReadOnly);
        m_objModel.push_back(model);
    };

    vkpbr::CommandPool cmd_pool(m_device, m_graphicsQueueIndex);
    auto               cmd_buffer = cmd_pool.MakeCmdBuffer();

    enum MtlIndex {kLight, kWhite, kRed, kGreen, kCount};
    universal_materials_.resize(MtlIndex::kCount);
    std::vector<int> mtl_indices;

    // light
    positions = {
        {343.0, 548.8, 227.0}, {343.0, 548.8, 332.0}, {213.0, 548.8, 332.0}, {213.0, 548.8, 227.0}};
    for (auto& p : positions)
        p.y -= 0.8f;
    mtl.diffuse  = {1.0, 1.0, 1.0};
    mtl.emission = {10.0, 10.0, 10.0};
    AddMesh(positions, {0, 1, 2, 0, 2, 3}, mtl, MtlIndex::kLight, cmd_buffer);

    universal_materials_[MtlIndex::kLight] = mtl;
    mtl_indices.push_back(kLight);

    {  // Floor, back wall and ceiling. Share the same material.
        positions = {{552.8, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 559.2}, {549.6, 0.0, 559.2}};

        mtl.diffuse  = {1.0, 1.0, 1.0};
        mtl.emission = {0.0, 0.0, 0.0};
        universal_materials_[MtlIndex::kWhite] = mtl;
        AddMesh(positions, {0, 1, 2, 0, 2, 3}, mtl, MtlIndex::kWhite, cmd_buffer);

        // ceiling
        positions = {
            {556.0, 548.8, 0.0}, {556.0, 548.8, 559.2}, {0.0, 548.8, 559.2}, {0.0, 548.8, 0.0}};
        AddMesh(positions, {0, 1, 2, 0, 2, 3}, mtl, MtlIndex::kWhite, cmd_buffer);
        // back wall
        positions = {
            {549.6, 0.0, 559.2}, {0.0, 0.0, 559.2}, {0.0, 548.8, 559.2}, {556.0, 548.8, 559.2}};

        AddMesh(positions, {0, 1, 2, 0, 2, 3}, mtl, MtlIndex::kWhite, cmd_buffer);
        mtl_indices.insert(mtl_indices.end(), {kWhite, kWhite, kWhite});
    }

    // right wall
    positions = {{0.0, 0.0, 559.2}, {0.0, 0.0, 0.0}, {0.0, 548.8, 0.0}, {0.0, 548.8, 559.2}};
    mtl.diffuse = {0.1, 0.9, 0.1};
    universal_materials_[MtlIndex::kGreen] = mtl;
    AddMesh(positions, {0, 1, 2, 0, 2, 3}, mtl, MtlIndex::kGreen, cmd_buffer);
    // left wall
    positions = {
        {552.8, 0.0, 0.0}, {549.6, 0.0, 559.2}, {556.0, 548.8, 559.2}, {556.0, 548.8, 0.0}};
    mtl.diffuse = {0.9, 0.1, 0.1};
    universal_materials_[MtlIndex::kRed] = mtl;
    AddMesh(positions, {0, 1, 2, 0, 2, 3}, mtl, MtlIndex::kRed, cmd_buffer);
    mtl_indices.insert(mtl_indices.end(), {kGreen, kRed});

    // short block and tall block both are white
    mtl.diffuse = {1.0, 1.0, 1.0};
    // short block
    positions = {{130.0, 165.0, 65.0},  {82.0, 165.0, 225.0},  {240.0, 165.0, 272.0},
                 {290.0, 165.0, 114.0}, {290.0, 0.0, 114.0},   {290.0, 165.0, 114.0},
                 {240.0, 165.0, 272.0}, {240.0, 0.0, 272.0},   {130.0, 0.0, 65.0},
                 {130.0, 165.0, 65.0},  {290.0, 165.0, 114.0}, {290.0, 0.0, 114.0},
                 {82.0, 0.0, 225.0},    {82.0, 165.0, 225.0},  {130.0, 165.0, 65.0},
                 {130.0, 0.0, 65.0},    {240.0, 0.0, 272.0},   {240.0, 165.0, 272.0},
                 {82.0, 165.0, 225.0},  {82.0, 0.0, 225.0}};
    for (int i = 0; i < positions.size(); i += 4) {
        std::vector<glm::vec3> face_positions;
        face_positions.assign(positions.begin() + i, positions.begin() + i + 4);
        AddMesh(face_positions, {0, 1, 2, 0, 2, 3}, mtl, MtlIndex::kWhite, cmd_buffer);
        mtl_indices.push_back(kWhite);
    }
    // tall block
    positions = {{423.0, 330.0, 247.0}, {265.0, 330.0, 296.0}, {314.0, 330.0, 456.0},
                 {472.0, 330.0, 406.0}, {423.0, 0.0, 247.0},   {423.0, 330.0, 247.0},
                 {472.0, 330.0, 406.0}, {472.0, 0.0, 406.0},   {472.0, 0.0, 406.0},
                 {472.0, 330.0, 406.0}, {314.0, 330.0, 456.0}, {314.0, 0.0, 456.0},
                 {314.0, 0.0, 456.0},   {314.0, 330.0, 456.0}, {265.0, 330.0, 296.0},
                 {265.0, 0.0, 296.0},   {265.0, 0.0, 296.0},   {265.0, 330.0, 296.0},
                 {423.0, 330.0, 247.0}, {423.0, 0.0, 247.0}};
    for (int i = 0; i < positions.size(); i += 4) {
        std::vector<glm::vec3> face_positions;
        face_positions.assign(positions.begin() + i, positions.begin() + i + 4);
        AddMesh(face_positions, {0, 1, 2, 0, 2, 3}, mtl, MtlIndex::kWhite, cmd_buffer);
        mtl_indices.push_back(kWhite);
    }
    CHECK_EQ(mtl_indices.size(), m_objModel.size());
    for (int i = 0; i < m_objModel.size(); ++i) {
        ObjInstance instance;
        instance.objIndex = i;
        instance.mtl_index = mtl_indices[i];
        m_objInstance.push_back(instance);
    }
    createTextureImages(cmd_buffer, {});
    
    // Creates the shader-storage buffer for the universal materials.
    materials_buffer_ =
        allocator_.MakeBuffer(cmd_buffer, universal_materials_, vkBU::eStorageBuffer);

    cmd_pool.SubmitAndWait(cmd_buffer);
    allocator_.ReleaseAllStagingBuffers();
}

//--------------------------------------------------------------------------------------------------
// Creating the uniform buffer holding the camera matrices
// - Buffer is host visible
//
void HelloVulkan::createUniformBuffer()
{
    using vkBU = vk::BufferUsageFlagBits;
    using vkMP = vk::MemoryPropertyFlagBits;

    m_cameraMat =
        allocator_.MakeBuffer(sizeof(CameraMatrices), vkBU::eUniformBuffer | vkBU::eTransferDst,
                              vkMP::eDeviceLocal);
    m_debug.setObjectName(m_cameraMat.handle, "cameraMat");
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
    vkpbr::CommandPool cmdGen(m_device, m_graphicsQueueIndex);

    auto cmdBuf = cmdGen.MakeCmdBuffer();
    m_sceneDesc = allocator_.MakeBuffer(cmdBuf, m_objInstance, vkBU::eStorageBuffer);
    cmdGen.SubmitAndWait(cmdBuf);
    allocator_.ReleaseAllStagingBuffers();
    // m_alloc.finalizeAndReleaseStaging();
    m_debug.setObjectName(m_sceneDesc.handle, "sceneDesc");
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
    if (textures.empty() && m_textures.empty()) {
        vkpbr::UniqueMemoryTexture texture;

        std::array<uint8_t, 4> color{255u, 255u, 255u, 255u};
        vk::DeviceSize         bufferSize      = sizeof(color);
        auto                   imgSize         = vk::Extent2D(1, 1);
        auto                   imageCreateInfo = vkpbr::MakeImage2DCreateInfo(imgSize, format);

        // Creating the dummy texture
        vkpbr::UniqueMemoryImage image =
            allocator_.MakeImage(cmdBuf, bufferSize, color.data(), imageCreateInfo);
        vk::ImageViewCreateInfo ivInfo =
            vkpbr::MakeImageViewCreateInfo(image.handle, imageCreateInfo);
        texture = allocator_.MakeTexture(image, ivInfo, samplerCreateInfo);

        // The image format must be in VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL
        vkpbr::CmdBarrierImageLayout(cmdBuf, texture.handle, vk::ImageLayout::eUndefined,
                                     vk::ImageLayout::eShaderReadOnlyOptimal);
        m_textures.push_back(texture);
    } else {
        // Uploading all images
        for (const auto& texture : textures) {
            std::stringstream o;
            int               texWidth, texHeight, texChannels;
            o << "media/textures/" << texture;
            std::string txtFile = io::FindFile(o.str(), defaultSearchPaths);

            stbi_uc* stbi_pixels =
                stbi_load(txtFile.c_str(), &texWidth, &texHeight, &texChannels, STBI_rgb_alpha);

            std::array<stbi_uc, 4> color{255u, 0u, 255u, 255u};

            stbi_uc* pixels = stbi_pixels;
            // Handle failure
            if (!stbi_pixels) {
                texWidth = texHeight = 1;
                texChannels          = 4;
                pixels               = reinterpret_cast<stbi_uc*>(color.data());
            }

            vk::DeviceSize bufferSize =
                static_cast<uint64_t>(texWidth) * texHeight * sizeof(uint8_t) * 4;
            auto imgSize = vk::Extent2D(texWidth, texHeight);
            auto imageCreateInfo =
                vkpbr::MakeImage2DCreateInfo(imgSize, format, vkIU::eSampled, true);

            {
                vkpbr::UniqueMemoryImage image =
                    allocator_.MakeImage(cmdBuf, bufferSize, pixels, imageCreateInfo);
                vkpbr::CmdGenerateMipmaps(cmdBuf, image.handle, format, imgSize,
                                          imageCreateInfo.mipLevels);
                vk::ImageViewCreateInfo ivInfo =
                    vkpbr::MakeImageViewCreateInfo(image.handle, imageCreateInfo);
                auto texture = allocator_.MakeTexture(image, ivInfo, samplerCreateInfo);

                m_textures.push_back(texture);
            }

            stbi_image_free(stbi_pixels);
        }
    }
}

//--------------------------------------------------------------------------------------------------
// Destroying all allocations
//
void HelloVulkan::destroyResources()
{
    m_device.destroy(m_graphicsPipeline);
    m_device.destroy(m_pipelineLayout);
    m_device.destroy(m_descPool);
    m_device.destroy(m_descSetLayout);
    m_cameraMat.DestroyFrom(m_device);
    m_sceneDesc.DestroyFrom(m_device);
    materials_buffer_.DestroyFrom(m_device);

    for (auto& m : m_objModel) {
        m.vertexBuffer.DestroyFrom(m_device);
        m.indexBuffer.DestroyFrom(m_device);
    }

    for (auto& t : m_textures) {
        // m_alloc.destroy(t);
        t.DestroyFrom(m_device);
    }

    //#Post
    m_device.destroy(m_postPipeline);
    m_device.destroy(m_postPipelineLayout);
    m_device.destroy(m_postDescPool);
    m_device.destroy(m_postDescSetLayout);
    m_offscreenColor.DestroyFrom(m_device);
    m_offscreenDepth.DestroyFrom(m_device);
    m_device.destroy(m_offscreenRenderPass);
    m_device.destroy(m_offscreenFramebuffer);

    // #VKRay
    m_rtBuilder.destroy();
    m_device.destroy(m_rtDescPool);
    m_device.destroy(m_rtDescSetLayout);
    m_device.destroy(m_rtPipeline);
    m_device.destroy(m_rtPipelineLayout);
    m_rtSBTBuffer.DestroyFrom(m_device);
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
    for (int i = 0; i < m_objInstance.size(); ++i) {
        auto& inst                = m_objInstance[i];
        auto& model               = m_objModel[inst.objIndex];
        m_pushConstant.instanceId = i;  // Telling which instance is drawn
        cmdBuf.pushConstants<ObjPushConstant>(m_pipelineLayout, vkSS::eVertex | vkSS::eFragment, 0,
                                              m_pushConstant);

        cmdBuf.bindVertexBuffers(0, {model.vertexBuffer.handle}, {offset});
        cmdBuf.bindIndexBuffer(model.indexBuffer.handle, 0, vk::IndexType::eUint32);
        cmdBuf.drawIndexed(model.num_indices, 1, 0, 0, 0);
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
    m_offscreenColor.DestroyFrom(m_device);
    m_offscreenDepth.DestroyFrom(m_device);

    // Creating the color image
    {
        auto colorCreateInfo = vkpbr::MakeImage2DCreateInfo(m_size, m_offscreenColorFormat,
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
        vkpbr::MakeImage2DCreateInfo(m_size, m_offscreenDepthFormat,
                                     vk::ImageUsageFlagBits::eDepthStencilAttachment);
    {
        vkpbr::UniqueMemoryImage image = allocator_.MakeImage(depthCreateInfo);

        vk::ImageViewCreateInfo depthStencilView;
        depthStencilView.setViewType(vk::ImageViewType::e2D);
        depthStencilView.setFormat(m_offscreenDepthFormat);
        depthStencilView.setSubresourceRange({vk::ImageAspectFlagBits::eDepth, 0, 1, 0, 1});
        depthStencilView.setImage(image.handle);

        m_offscreenDepth = allocator_.MakeTexture(image, depthStencilView);
    }

    // Setting the image layout for both color and depth
    {
        vkpbr::CommandPool genCmdBuf(m_device, m_graphicsQueueIndex);
        auto               cmdBuf = genCmdBuf.MakeCmdBuffer();
        vkpbr::CmdBarrierImageLayout(cmdBuf, m_offscreenColor.handle, vk::ImageLayout::eUndefined,
                                     vk::ImageLayout::eGeneral);
        vkpbr::CmdBarrierImageLayout(cmdBuf, m_offscreenDepth.handle, vk::ImageLayout::eUndefined,
                                     vk::ImageLayout::eDepthStencilAttachmentOptimal,
                                     vk::ImageAspectFlagBits::eDepth);

        genCmdBuf.SubmitAndWait(cmdBuf);
    }

    // Creating a renderpass for the offscreen
    if (!m_offscreenRenderPass) {
        m_offscreenRenderPass =
            vkpbr::MakeRenderPass(m_device, {m_offscreenColorFormat}, m_offscreenDepthFormat, 1,
                                  true, true, vk::ImageLayout::eGeneral, vk::ImageLayout::eGeneral);
    }

    // Creating the frame buffer for offscreen
    std::vector<vk::ImageView> attachments = {m_offscreenColor.descriptor.imageView,
                                              m_offscreenDepth.descriptor.imageView};

    m_device.destroy(m_offscreenFramebuffer);
    vk::FramebufferCreateInfo info;
    info.setRenderPass(m_offscreenRenderPass);
    info.setAttachments(attachments);
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
    vk::PushConstantRange pushConstantRanges = {vk::ShaderStageFlagBits::eFragment, 0,
                                                sizeof(float)};

    // Creating the pipeline layout
    vk::PipelineLayoutCreateInfo pipelineLayoutCreateInfo;
    pipelineLayoutCreateInfo.setSetLayouts(m_postDescSetLayout);
    pipelineLayoutCreateInfo.setPushConstantRanges(pushConstantRanges);
    m_postPipelineLayout = m_device.createPipelineLayout(pipelineLayoutCreateInfo);

    // Pipeline: completely generic, no vertices
    std::vector<std::string> paths = defaultSearchPaths;

    nvvk::GraphicsPipelineGeneratorCombined pipelineGenerator(m_device, m_postPipelineLayout,
                                                              m_renderPass);
    pipelineGenerator.addShader(io::LoadBinaryFile("shaders/passthrough.vert.spv", paths),
                                vk::ShaderStageFlagBits::eVertex);
    pipelineGenerator.addShader(io::LoadBinaryFile("shaders/post.frag.spv", paths),
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

    m_postDescSetLayoutBind.AddBinding(vkDS(0, vkDT::eCombinedImageSampler, 1, vkSS::eFragment));
    m_postDescSetLayout = m_postDescSetLayoutBind.MakeLayout(m_device);
    m_postDescPool      = m_postDescSetLayoutBind.MakePool(m_device);

    auto desc_set_alloc_info = vk::DescriptorSetAllocateInfo()
                                   .setDescriptorPool(m_postDescPool)
                                   .setSetLayouts(m_postDescSetLayout);
    m_postDescSet = m_device.allocateDescriptorSets(desc_set_alloc_info).front();

    // m_postDescSet = nvvk::allocateDescriptorSet(m_device, m_postDescPool, m_postDescSetLayout);
}

//--------------------------------------------------------------------------------------------------
// Update the output
//
void HelloVulkan::updatePostDescriptorSet()
{
    vk::DescriptorImageInfo dii = m_offscreenColor.descriptor;
    vk::WriteDescriptorSet  writeDescriptorSets =
        m_postDescSetLayoutBind.MakeWrite(m_postDescSet, 0, &dii);
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

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

//--------------------------------------------------------------------------------------------------
// Initialize Vulkan ray tracing
// #VKRay
void HelloVulkan::initRayTracing()
{
    // Requesting ray tracing properties
    auto properties =
        m_physicalDevice.getProperties2<vk::PhysicalDeviceProperties2,
                                        vk::PhysicalDeviceRayTracingPipelinePropertiesKHR>();
    m_rtProperties = properties.get<vk::PhysicalDeviceRayTracingPipelinePropertiesKHR>();
    m_rtBuilder.setup(m_device, &allocator_, m_graphicsQueueIndex);
}

//--------------------------------------------------------------------------------------------------
// Convert an OBJ model into the ray tracing geometry used to build the BLAS
//
vkpbr::RaytracingBuilderKHR::BlasInput HelloVulkan::objectToVkGeometryKHR(const ObjModel& model)
{
    // BLAS builder requires raw device addresses.
    vk::DeviceAddress vertexAddress = m_device.getBufferAddress({model.vertexBuffer.handle});
    vk::DeviceAddress indexAddress  = m_device.getBufferAddress({model.indexBuffer.handle});

    uint32_t maxPrimitiveCount = model.num_indices / 3;

    // Describe buffer as array of VertexObj.
    vk::AccelerationStructureGeometryTrianglesDataKHR triangles;
    triangles.setVertexFormat(vk::Format::eR32G32B32Sfloat);  // vec3 vertex position data.
    triangles.setVertexData(vertexAddress);
    triangles.setVertexStride(sizeof(vkpbr::VertexData));
    // Describe index data (32-bit unsigned int)
    triangles.setIndexType(vk::IndexType::eUint32);
    triangles.setIndexData(indexAddress);
    // Indicate identity transform by setting transformData to null device pointer.
    triangles.setTransformData({});
    triangles.setMaxVertex(model.num_vertices);

    // Identify the above data as containing opaque triangles.
    vk::AccelerationStructureGeometryKHR asGeom;
    asGeom.setGeometryType(vk::GeometryTypeKHR::eTriangles);
    asGeom.setFlags(vk::GeometryFlagBitsKHR::eOpaque);
    asGeom.geometry.setTriangles(triangles);

    // The entire array will be used to build the BLAS.
    vk::AccelerationStructureBuildRangeInfoKHR offset;
    offset.setFirstVertex(0);
    offset.setPrimitiveCount(maxPrimitiveCount);
    offset.setPrimitiveOffset(0);
    offset.setTransformOffset(0);

    // Our blas is made from only one geometry, but could be made of many geometries
    vkpbr::RaytracingBuilderKHR::BlasInput input;
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
    std::vector<vkpbr::RaytracingBuilderKHR::BlasInput> allBlas;
    allBlas.reserve(m_objModel.size());
    for (const auto& obj : m_objModel) {
        auto blas = objectToVkGeometryKHR(obj);

        // We could add more geometry in each BLAS, but we add only one for now
        allBlas.emplace_back(blas);
    }
    m_rtBuilder.buildBlas(allBlas, vkrt::BuildBvhFlagBits::ePreferFastTrace);
}

void HelloVulkan::createTopLevelAS()
{
    std::vector<vkpbr::RaytracingBuilderKHR::Instance> tlas;
    tlas.reserve(m_objInstance.size());
    for (int i = 0; i < static_cast<int>(m_objInstance.size()); i++) {
        vkpbr::RaytracingBuilderKHR::Instance rayInst;
        rayInst.transform  = m_objInstance[i].transform;  // Position of the instance
        rayInst.instanceId = i;                           // gl_InstanceID
        rayInst.blasId     = m_objInstance[i].objIndex;
        rayInst.hitGroupId = 0;  // We will use the same hit group for all objects
        rayInst.flags      = vk::GeometryInstanceFlagBitsKHR::eTriangleFacingCullDisable;
        tlas.emplace_back(rayInst);
    }
    m_rtBuilder.buildTlas(tlas, vk::BuildAccelerationStructureFlagBitsKHR::ePreferFastTrace);
}

//--------------------------------------------------------------------------------------------------
// This descriptor set holds the Acceleration structure and the output image
//
void HelloVulkan::createRtDescriptorSet()
{
    using vkDT   = vk::DescriptorType;
    using vkSS   = vk::ShaderStageFlagBits;
    using vkDSLB = vk::DescriptorSetLayoutBinding;

    m_rtDescSetLayoutBind.AddBinding(vkDSLB(0, vkDT::eAccelerationStructureKHR, 1,
                                            vkSS::eRaygenKHR | vkSS::eClosestHitKHR));  // TLAS
    m_rtDescSetLayoutBind.AddBinding(
        vkDSLB(1, vkDT::eStorageImage, 1, vkSS::eRaygenKHR));  // Output
                                                               // image

    m_rtDescPool      = m_rtDescSetLayoutBind.MakePool(m_device);
    m_rtDescSetLayout = m_rtDescSetLayoutBind.MakeLayout(m_device);
    m_rtDescSet       = m_device.allocateDescriptorSets({m_rtDescPool, 1, &m_rtDescSetLayout})[0];

    vk::AccelerationStructureKHR                   tlas = m_rtBuilder.getAccelerationStructure();
    vk::WriteDescriptorSetAccelerationStructureKHR descASInfo;
    descASInfo.setAccelerationStructureCount(1);
    descASInfo.setPAccelerationStructures(&tlas);
    vk::DescriptorImageInfo imageInfo{
        {}, m_offscreenColor.descriptor.imageView, vk::ImageLayout::eGeneral};

    std::vector<vk::WriteDescriptorSet> writes;
    writes.emplace_back(m_rtDescSetLayoutBind.MakeWrite(m_rtDescSet, 0, &descASInfo));
    writes.emplace_back(m_rtDescSetLayoutBind.MakeWrite(m_rtDescSet, 1, &imageInfo));
    m_device.updateDescriptorSets(static_cast<uint32_t>(writes.size()), writes.data(), 0, nullptr);
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
    m_device.updateDescriptorSets(wds, nullptr);
}

static vkrt::RtShaderGroupCreateInfo MakeEmptyShaderGroupCI() {
    return vkrt::RtShaderGroupCreateInfo()
        .setType(vkrt::RtShaderGroupType::eGeneral)
        .setAnyHitShader(vkrt::kShaderUnused)
        .setClosestHitShader(vkrt::kShaderUnused)
        .setGeneralShader(vkrt::kShaderUnused)
        .setIntersectionShader(vkrt::kShaderUnused);
}

//--------------------------------------------------------------------------------------------------
// Pipeline for the ray tracer: all shaders, raygen, chit, miss
//
void HelloVulkan::createRtPipeline()
{
    std::vector<std::string> paths = defaultSearchPaths;

    vk::ShaderModule raygenSM =
        vkpbr::MakeShaderModule(m_device, io::LoadBinaryFile("shaders/raytrace.rgen.spv", paths));
    vk::ShaderModule missSM =
        vkpbr::MakeShaderModule(m_device, io::LoadBinaryFile("shaders/raytrace.rmiss.spv", paths));

    // The second miss shader is invoked when a shadow ray misses the geometry. It
    // simply indicates that no occlusion has been found
    vk::ShaderModule shadowmissSM =
        vkpbr::MakeShaderModule(m_device,
                                io::LoadBinaryFile("shaders/raytraceShadow.rmiss.spv", paths));
    vk::ShaderModule chitSM =
        vkpbr::MakeShaderModule(m_device, io::LoadBinaryFile("shaders/raytrace.rchit.spv", paths));

    std::vector<vk::PipelineShaderStageCreateInfo> stages(RtStages::kNumStages);
    stages[RtStages::kRaygen]     = {{}, vk::ShaderStageFlagBits::eRaygenKHR, raygenSM, "main"};
    stages[RtStages::kMiss]       = {{}, vk::ShaderStageFlagBits::eMissKHR, missSM, "main"};
    stages[RtStages::kShadowMiss] = {{}, vk::ShaderStageFlagBits::eMissKHR, shadowmissSM, "main"};
    stages[RtStages::kCHit]       = {{}, vk::ShaderStageFlagBits::eClosestHitKHR, chitSM, "main"};

    // Raygen
    auto rg = MakeEmptyShaderGroupCI().setType(vkrt::RtShaderGroupType::eGeneral);
    rg.setGeneralShader(RtStages::kRaygen);
    m_rtShaderGroups.push_back(rg);
    // Miss
    auto mg = MakeEmptyShaderGroupCI().setType(vk::RayTracingShaderGroupTypeKHR::eGeneral);
    mg.setGeneralShader(RtStages::kMiss);
    m_rtShaderGroups.push_back(mg);
    // Shadow Miss
    mg.setGeneralShader(RtStages::kShadowMiss);
    m_rtShaderGroups.push_back(mg);
    // Hit Group - Closest Hit + AnyHit
    auto hg = MakeEmptyShaderGroupCI().setType(vkrt::RtShaderGroupType::eTrianglesHitGroup);
    hg.setClosestHitShader(RtStages::kCHit);
    m_rtShaderGroups.push_back(hg);

    // RT pipeline 1.1: pipeline layout - push constant.
    // --------------------------------------------------------------------------------------------
    vk::PipelineLayoutCreateInfo pipelineLayoutCreateInfo;
    // Push constant: we want to be able to update constants used by the shaders
    vk::PushConstantRange pushConstant{vk::ShaderStageFlagBits::eRaygenKHR
                                           | vk::ShaderStageFlagBits::eClosestHitKHR
                                           | vk::ShaderStageFlagBits::eMissKHR,
                                       0, sizeof(RtPushConstant)};
    pipelineLayoutCreateInfo.setPushConstantRanges(pushConstant);

    // RT pipeline 1.2: pipeline layout - descriptor set layouts.
    // --------------------------------------------------------------------------------------------
    // One specific to ray tracing (index 0), and one shared with the raster pipeline (index 1).
    std::vector<vk::DescriptorSetLayout> rtDescSetLayouts = {m_rtDescSetLayout, m_descSetLayout};
    pipelineLayoutCreateInfo.setSetLayouts(rtDescSetLayouts);

    m_rtPipelineLayout = m_device.createPipelineLayout(pipelineLayoutCreateInfo);

    // RT pipeline 2, 3, 4: shader stages, shader groups, and recursion depth.
    // --------------------------------------------------------------------------------------------
    vk::RayTracingPipelineCreateInfoKHR rayPipelineInfo;
    rayPipelineInfo.setStages(stages);
    // In this case, m_rtShaderGroups.size() == 4: we have 1 raygen group, 2 miss shader groups, and
    // one hit group.
    rayPipelineInfo.setGroups(m_rtShaderGroups);
    rayPipelineInfo.setMaxPipelineRayRecursionDepth(8);  // Ray depth
    rayPipelineInfo.setLayout(m_rtPipelineLayout);
    m_rtPipeline = m_device.createRayTracingPipelineKHR({}, {}, rayPipelineInfo).value;

    m_device.destroy(raygenSM);
    m_device.destroy(missSM);
    m_device.destroy(shadowmissSM);
    m_device.destroy(chitSM);
}

static inline u32 AlignUp(u32 x, u32 alignment)
{
    x += alignment - 1;

    if ((alignment & (alignment - 1)) == 0) {
        u32 answer1 = (x / alignment) * alignment;
        u32 answer2 = x & ~(alignment - 1);
        assert(answer1 == answer2);
        return answer2;
    }

    return (x / alignment) * alignment;
}

//--------------------------------------------------------------------------------------------------
// The Shader Binding Table (SBT)
// - getting all shader handles and write them in a SBT buffer
// - Besides exception, this could be always done like this
//   See how the SBT buffer is used in run()
//
void HelloVulkan::createRtShaderBindingTable()
{
    auto groupCount = static_cast<uint32_t>(m_rtShaderGroups.size());  // 4 shaders: raygen, 2 miss,
                                                                       // chit
    uint32_t groupHandleSize = m_rtProperties.shaderGroupHandleSize;   // Size of a program
                                                                       // identifier
    // Compute the actual size needed per SBT entry (round-up to alignment needed).
    uint32_t groupSizeAligned = AlignUp(groupHandleSize, m_rtProperties.shaderGroupBaseAlignment);
    // Bytes needed for the SBT.
    uint32_t sbtSize = groupCount * groupSizeAligned;

    // Fetch all the shader handles used in the pipeline. This is opaque data,
    // so we store it in a vector of bytes.
    std::vector<uint8_t> shaderHandleStorage(sbtSize);
    auto result = m_device.getRayTracingShaderGroupHandlesKHR(m_rtPipeline, 0, groupCount, sbtSize,
                                                              shaderHandleStorage.data());
    assert(result == vk::Result::eSuccess);

    // Allocate a buffer for storing the SBT. Give it a debug name for NSight.
    m_rtSBTBuffer = allocator_.MakeBuffer(
        sbtSize,
        vk::BufferUsageFlagBits::eTransferSrc | vk::BufferUsageFlagBits::eShaderDeviceAddress
            | vk::BufferUsageFlagBits::eShaderBindingTableKHR,
        vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent);
    m_debug.setObjectName(m_rtSBTBuffer.handle, std::string("SBT").c_str());

    // Map the SBT buffer and write in the handles.
    void* mapped = m_device.mapMemory(m_rtSBTBuffer.memory, 0, VK_WHOLE_SIZE);
    auto* pData  = reinterpret_cast<uint8_t*>(mapped);
    for (uint32_t g = 0; g < groupCount; g++) {
        memcpy(pData, shaderHandleStorage.data() + g * groupHandleSize, groupHandleSize);
        pData += groupSizeAligned;
    }
    m_device.unmapMemory(m_rtSBTBuffer.memory);
    allocator_.ReleaseAllStagingBuffers();
}

//--------------------------------------------------------------------------------------------------
// Ray Tracing the scene
//
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

    // Size of a program identifier
    uint32_t groupSize =
        AlignUp(m_rtProperties.shaderGroupHandleSize, m_rtProperties.shaderGroupBaseAlignment);
    uint32_t          groupStride = groupSize;
    vk::DeviceAddress sbtAddress  = m_device.getBufferAddress({m_rtSBTBuffer.handle});

    using Stride = vk::StridedDeviceAddressRegionKHR;
    std::array<Stride, 4> stride_addrs{
        Stride{sbtAddress + 0u * groupSize, groupStride, groupSize * 1},  // raygen
        Stride{sbtAddress + 1u * groupSize, groupStride, groupSize * 2},  // miss
        Stride{sbtAddress + 3u * groupSize, groupStride, groupSize * 1},  // hit
        Stride{0u, 0u, 0u}};                                              // callable

    cmdBuf.traceRaysKHR(&stride_addrs[0], &stride_addrs[1], &stride_addrs[2], &stride_addrs[3],
                        m_size.width, m_size.height, 1);

    m_debug.endLabel(cmdBuf);
}

void HelloVulkan::UpdateFrame()
{
    static glm::mat4 cached_camera_view{1.0f};
    static float     cached_camera_fov{60.0f};
    static glm::ivec2 cached_framebuffer_size;

    glm::ivec2 current_framebuffer_size;
    glfwGetFramebufferSize(m_window, &current_framebuffer_size.x, &current_framebuffer_size.y);
    glm::mat4 current_view = camera_->ViewMatrix();
    float     current_fov  = camera_->Fov();

    if (current_view == cached_camera_view && cached_camera_fov == current_fov
        && cached_framebuffer_size == current_framebuffer_size) {
        ++m_rtPushConstants.accumulated_frames;
    } else {
        ResetFrame();
        cached_camera_view = current_view;
        cached_camera_fov  = current_fov;
        cached_framebuffer_size = current_framebuffer_size;
    }
}

void HelloVulkan::ResetFrame()
{
    m_rtPushConstants.accumulated_frames = -1;
}
