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

/**

# class nvvk::RaytracingBuilderKHR

Base functionality of raytracing

This class does not implement all what you need to do raytracing, but
helps creating the BLAS and TLAS, which then can be used by different
raytracing usage.

# Setup and Usage
~~~~ C++
m_rtBuilder.setup(device, memoryAllocator, queueIndex);
// Create array of VkGeometryNV
m_rtBuilder.buildBlas(allBlas);
// Create array of RaytracingBuilder::instance
m_rtBuilder.buildTlas(instances);
// Retrieve the acceleration structure
const VkAccelerationStructureNV& tlas = m.rtBuilder.getAccelerationStructure()
~~~~
*/


#include <iostream>
#include <mutex>
#include <vulkan/vulkan.h>

#include "nvh/nvprint.hpp"
#include "nvmath/nvmath.h"

#include "nvh/nvprint.hpp"

#include "../../vk_raytracing_tutorial_KHR/ray_tracing_intersection/vk_utils.h"

namespace vkpbr {
struct RaytracingBuilderKHR {
    RaytracingBuilderKHR(RaytracingBuilderKHR const&) = delete;
    RaytracingBuilderKHR& operator=(RaytracingBuilderKHR const&) = delete;

    RaytracingBuilderKHR() = default;

    // Bottom-level acceleration structure
    struct Blas {
        UniqueMemoryAccelStruct                as;     // VkAccelerationStructureKHR
        vk::BuildAccelerationStructureFlagsKHR flags;  // specifying additional parameters for
                                                       // acceleration structure builds
        std::vector<vk::AccelerationStructureCreateGeometryTypeInfoKHR>
            asCreateGeometryInfo;  // specifies the shape of geometries that will be
                                   // built into an acceleration structure
        std::vector<vk::AccelerationStructureGeometryKHR> asGeometry;  // data used to build
                                                                       // acceleration structure
                                                                       // geometry
        std::vector<vk::AccelerationStructureBuildOffsetInfoKHR> asBuildOffsetInfo;
    };

    //--------------------------------------------------------------------------------------------------
    // Initializing the allocator and querying the raytracing properties
    //
    void setup(const VkDevice& device, UniqueMemoryAllocator* allocator, uint32_t queueIndex)
    {
        m_device     = device;
        m_queueIndex = queueIndex;
        m_debug.setup(device);
        m_alloc = allocator;
    }

    // This is an instance of a BLAS
    struct Instance {
        uint32_t                     blasId{0};      // Index of the BLAS in m_blas
        uint32_t                     instanceId{0};  // Instance Index (gl_InstanceID)
        uint32_t                     hitGroupId{0};  // Hit group index in the SBT
        uint32_t                     mask{0xFF};  // Visibility mask, will be AND-ed with ray mask
        vk::GeometryInstanceFlagsKHR flags{
            VK_GEOMETRY_INSTANCE_TRIANGLE_FACING_CULL_DISABLE_BIT_KHR};
        glm::mat4 transform{glm::mat4(1)};  // Identity
    };

    //--------------------------------------------------------------------------------------------------
    // Destroying all allocations
    //
    void destroy()
    {
        for (auto& b : m_blas) {
            b.as.DestroyFrom(m_device);
        }
        m_tlas.as.DestroyFrom(m_device);
        m_instBuffer.DestroyFrom(m_device);
        m_blas.clear();
        m_tlas = {};
    }

    // Returning the constructed top-level acceleration structure
    vk::AccelerationStructureKHR getAccelerationStructure() const { return m_tlas.as.handle; }

    //--------------------------------------------------------------------------------------------------
    // Create all the BLAS from the vector of vectors of VkGeometryNV
    // - There will be one BLAS per vector of VkGeometryNV
    // - There will be as many BLAS there are items in the geoms vector
    // - The resulting BLAS are stored in m_blas
    //
    void buildBlas(const std::vector<RaytracingBuilderKHR::Blas>& blas_,
                   vk::BuildAccelerationStructureFlagsKHR         flags =
                       vk::BuildAccelerationStructureFlagBitsKHR::ePreferFastTrace)
    {
        m_blas = blas_;  // Keeping a copy

        vk::DeviceSize maxScratch{0};  // Largest scratch buffer for our BLAS

        // Is compaction requested?
        bool doCompaction =
            FlagsMatch(flags, vk::BuildAccelerationStructureFlagBitsKHR::eAllowCompaction);
        std::cout << "do compaction = " << (doCompaction ? "yes" : "no") << std::endl;
        std::vector<vk::DeviceSize> originalSizes;
        originalSizes.resize(m_blas.size());

        // Iterate over the groups of geometries, creating one BLAS for each group
        int idx{0};
        for (auto& blas : m_blas) {
            vk::AccelerationStructureCreateInfoKHR asCreateInfo;
            asCreateInfo.type             = vk::AccelerationStructureTypeKHR::eBottomLevel;
            asCreateInfo.flags            = flags;
            asCreateInfo.maxGeometryCount = (uint32_t)blas.asCreateGeometryInfo.size();
            asCreateInfo.pGeometryInfos   = blas.asCreateGeometryInfo.data();

            // Create an acceleration structure identifier and allocate memory to
            // store the resulting structure data
            blas.as = m_alloc->MakeAccelStruct(asCreateInfo);
            m_debug.setObjectName(blas.as.handle,
                                  (std::string("Blas" + std::to_string(idx)).c_str()));

            // Estimate the amount of scratch memory required to build the BLAS, and
            // update the size of the scratch buffer that will be allocated to
            // sequentially build all BLASes
            vk::AccelerationStructureMemoryRequirementsInfoKHR memoryRequirementsInfo;
            memoryRequirementsInfo.type =
                vk::AccelerationStructureMemoryRequirementsTypeKHR::eBuildScratch;
            memoryRequirementsInfo.accelerationStructure = blas.as.handle;
            memoryRequirementsInfo.buildType = vk::AccelerationStructureBuildTypeKHR::eDevice;

            vk::MemoryRequirements2 reqMem =
                m_device.getAccelerationStructureMemoryRequirementsKHR(memoryRequirementsInfo);
            // vkGetAccelerationStructureMemoryRequirementsKHR(m_device, &memoryRequirementsInfo,
            //&reqMem);
            vk::DeviceSize scratchSize = reqMem.memoryRequirements.size;
            std::cout << "scratch size: " << scratchSize << std::endl;

            blas.flags = flags;
            maxScratch = std::max(maxScratch, scratchSize);

            // Original size
            memoryRequirementsInfo.type =
                vk::AccelerationStructureMemoryRequirementsTypeKHR::eObject;
            // VK_ACCELERATION_STRUCTURE_MEMORY_REQUIREMENTS_TYPE_OBJECT_KHR;
            reqMem = m_device.getAccelerationStructureMemoryRequirementsKHR(memoryRequirementsInfo);
            // vkGetAccelerationStructureMemoryRequirementsKHR(m_device, &memoryRequirementsInfo,
            //&reqMem);
            originalSizes[idx] = reqMem.memoryRequirements.size;

            idx++;
        }

        for (vk::DeviceSize s : originalSizes) {
            std::cout << "original size: " << s << std::endl;
        }
        std::cout << "max_size = " << maxScratch << std::endl;

        // Allocate the scratch buffers holding the temporary data of the
        // acceleration structure builder

        UniqueMemoryBuffer scratchBuffer =
            m_alloc->MakeBuffer(maxScratch, vk::BufferUsageFlagBits::eRayTracingKHR
                                                | vk::BufferUsageFlagBits::eShaderDeviceAddress);
        vk::BufferDeviceAddressInfo bufferInfo{scratchBuffer.handle};
        vk::DeviceAddress           scratchAddress = m_device.getBufferAddress(bufferInfo);


        // Query size of compact BLAS
        vk::QueryPoolCreateInfo qpci;
        qpci.queryCount         = (uint32_t)m_blas.size();
        qpci.queryType          = vk::QueryType::eAccelerationStructureCompactedSizeKHR;
        vk::QueryPool queryPool = m_device.createQueryPool(qpci);
        // vkCreateQueryPool(m_device, &qpci, nullptr, &queryPool);


        // Create a command buffer containing all the BLAS builds
        // nvvk::CommandPool            genCmdBuf(m_device, m_queueIndex);
        vkpbr::CommandPool             genCmdBuf(m_device, m_queueIndex);
        int                            ctr{0};
        std::vector<vk::CommandBuffer> allCmdBufs;
        allCmdBufs.reserve(m_blas.size());
        for (auto& blas : m_blas) {
            vk::CommandBuffer cmdBuf = genCmdBuf.MakeCmdBuffer();
            allCmdBufs.push_back(cmdBuf);

            const vk::AccelerationStructureGeometryKHR*   pGeometry = blas.asGeometry.data();
            vk::AccelerationStructureBuildGeometryInfoKHR bottomASInfo;
            bottomASInfo.type                      = vk::AccelerationStructureTypeKHR::eBottomLevel;
            bottomASInfo.flags                     = flags;
            bottomASInfo.update                    = VK_FALSE;
            bottomASInfo.srcAccelerationStructure  = nullptr;
            bottomASInfo.dstAccelerationStructure  = blas.as.handle;
            bottomASInfo.geometryArrayOfPointers   = VK_FALSE;
            bottomASInfo.geometryCount             = (uint32_t)blas.asGeometry.size();
            bottomASInfo.ppGeometries              = &pGeometry;
            bottomASInfo.scratchData.deviceAddress = scratchAddress;

            // Pointers of offset
            std::vector<const vk::AccelerationStructureBuildOffsetInfoKHR*> pBuildOffset(
                blas.asBuildOffsetInfo.size());
            for (size_t i = 0; i < blas.asBuildOffsetInfo.size(); i++)
                pBuildOffset[i] = &blas.asBuildOffsetInfo[i];

            // Building the AS
            cmdBuf.buildAccelerationStructureKHR(bottomASInfo, pBuildOffset);

            // Since the scratch buffer is reused across builds, we need a barrier to ensure one
            // build is finished before starting the next one
            // VkMemoryBarrier barrier{VK_STRUCTURE_TYPE_MEMORY_BARRIER};
            // barrier.srcAccessMask = VK_ACCESS_ACCELERATION_STRUCTURE_WRITE_BIT_KHR;
            // barrier.dstAccessMask = VK_ACCESS_ACCELERATION_STRUCTURE_READ_BIT_KHR;
            // vkCmdPipelineBarrier(cmdBuf, VK_PIPELINE_STAGE_ACCELERATION_STRUCTURE_BUILD_BIT_KHR,
            //                     VK_PIPELINE_STAGE_ACCELERATION_STRUCTURE_BUILD_BIT_KHR, 0, 1,
            //                     &barrier, 0, nullptr, 0, nullptr);

            auto barrier_pp =
                vk::MemoryBarrier()
                    .setSrcAccessMask(vk::AccessFlagBits::eAccelerationStructureWriteKHR)
                    .setDstAccessMask(vk::AccessFlagBits::eAccelerationStructureReadKHR);
            cmdBuf.pipelineBarrier(vk::PipelineStageFlagBits::eAccelerationStructureBuildKHR,
                                   vk::PipelineStageFlagBits::eAccelerationStructureBuildKHR, {},
                                   barrier_pp, nullptr, nullptr);

            // Query the compact size
            if (doCompaction) {
                /*vkCmdWriteAccelerationStructuresPropertiesKHR(
                    cmdBuf, 1, reinterpret_cast<VkAccelerationStructureKHR*>(&blas.as.handle),
                    VK_QUERY_TYPE_ACCELERATION_STRUCTURE_COMPACTED_SIZE_KHR, queryPool, ctr++);*/
                cmdBuf.writeAccelerationStructuresPropertiesKHR(
                    blas.as.handle, vk::QueryType::eAccelerationStructureCompactedSizeKHR,
                    queryPool, ctr++);
            }
        }
        genCmdBuf.SubmitAndWait(allCmdBufs);
        allCmdBufs.clear();

        // Compacting all BLAS
        if (doCompaction) {
            vk::CommandBuffer cmdBuf = genCmdBuf.MakeCmdBuffer();

            // Get the size result back
            std::vector<VkDeviceSize> compactSizes(m_blas.size());
            //vkGetQueryPoolResults(m_device, queryPool, 0, (uint32_t)compactSizes.size(),
            //                      compactSizes.size() * sizeof(VkDeviceSize), compactSizes.data(),
            //                      sizeof(VkDeviceSize), VK_QUERY_RESULT_WAIT_BIT);
            m_device.getQueryPoolResults<vk::DeviceSize>(queryPool, 0, cast_u32(compactSizes.size()),
                                                         compactSizes,
                                         //DataSize(compactSizes), compactSizes.data(),
                                         sizeof(vk::DeviceSize), vk::QueryResultFlagBits::eWait);


            // Compacting
            std::vector<UniqueMemoryAccelStruct> cleanupAS(m_blas.size());
            uint32_t                             totOriginalSize{0}, totCompactSize{0};
            for (int i = 0; i < m_blas.size(); i++) {
                // LOGI("Reducing %i, from %d to %d \n", i, originalSizes[i], compactSizes[i]);
                totOriginalSize += (uint32_t)originalSizes[i];
                totCompactSize += (uint32_t)compactSizes[i];

                // Creating a compact version of the AS
                vk::AccelerationStructureCreateInfoKHR asCreateInfo;
                asCreateInfo.compactedSize = compactSizes[i];
                asCreateInfo.type          = vk::AccelerationStructureTypeKHR::eBottomLevel;
                asCreateInfo.flags         = flags;
                auto as                    = m_alloc->MakeAccelStruct(asCreateInfo);

                // Copy the original BLAS to a compact version
                vk::CopyAccelerationStructureInfoKHR copyInfo;
                copyInfo.src  = m_blas[i].as.handle;
                copyInfo.dst  = as.handle;
                copyInfo.mode = vk::CopyAccelerationStructureModeKHR::eCompact;
                cmdBuf.copyAccelerationStructureKHR(copyInfo);
                //vkCmdCopyAccelerationStructureKHR(cmdBuf, &copyInfo);
                cleanupAS[i] = m_blas[i].as;
                m_blas[i].as = as;
            }
            genCmdBuf.SubmitAndWait(cmdBuf);

            // Destroying the previous version
            for (auto as : cleanupAS)
                as.DestroyFrom(m_device);

            LOGI("------------------\n");
            LOGI("Reducing from: %u to: %u = %u (%2.2f%s smaller) \n", totOriginalSize,
                 totCompactSize, totOriginalSize - totCompactSize,
                 (totOriginalSize - totCompactSize) / float(totOriginalSize) * 100.f, "%%");
        }

        m_device.destroyQueryPool(queryPool);
        //vkDestroyQueryPool(m_device, queryPool, nullptr);
        scratchBuffer.DestroyFrom(m_device);
        m_alloc->ReleaseAllStagingBuffers();
    }

    //--------------------------------------------------------------------------------------------------
    // Convert an Instance object into a VkGeometryInstanceNV

    vk::AccelerationStructureInstanceKHR instanceToVkGeometryInstanceKHR(const Instance& instance)
    {
        Blas& blas{m_blas[instance.blasId]};

        vk::AccelerationStructureDeviceAddressInfoKHR addressInfo;
        addressInfo.accelerationStructure = blas.as.handle;
        vk::DeviceAddress blasAddress = m_device.getAccelerationStructureAddressKHR(addressInfo);

        vk::AccelerationStructureInstanceKHR gInst;
        // The matrices for the instance transforms are row-major, instead of
        // column-major in the rest of the application
        glm::mat4 transp = glm::transpose(instance.transform);
        // The gInst.transform value only contains 12 values, corresponding to a 4x3
        // matrix, hence saving the last row that is anyway always (0,0,0,1). Since
        // the matrix is row-major, we simply copy the first 12 values of the
        // original 4x4 matrix
        memcpy(&gInst.transform, &transp, sizeof(gInst.transform));
        gInst.instanceCustomIndex                    = instance.instanceId;
        gInst.mask                                   = instance.mask;
        gInst.instanceShaderBindingTableRecordOffset = instance.hitGroupId;
        gInst.setFlags(instance.flags);
        gInst.accelerationStructureReference = blasAddress;
        return gInst;
    }

    //--------------------------------------------------------------------------------------------------
    // Creating the top-level acceleration structure from the vector of Instance
    // - See struct of Instance
    // - The resulting TLAS will be stored in m_tlas
    //
    void buildTlas(const std::vector<Instance>&         instances,
                   VkBuildAccelerationStructureFlagsKHR flags =
                       VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR)
    {
        m_tlas.flags = flags;

        // The same structure we used in `HelloVulkan::object_to_vkGeometryKHR`.
        VkAccelerationStructureCreateGeometryTypeInfoKHR geometryCreate{
            VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_GEOMETRY_TYPE_INFO_KHR};
        geometryCreate.geometryType      = VK_GEOMETRY_TYPE_INSTANCES_KHR;
        geometryCreate.maxPrimitiveCount = (static_cast<uint32_t>(instances.size()));
        geometryCreate.allowsTransforms  = (VK_TRUE);

        // The same structure used in `RaytracingBuilderKHR::buildBlas`.
        VkAccelerationStructureCreateInfoKHR asCreateInfo{
            VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR};
        asCreateInfo.type             = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;
        asCreateInfo.flags            = flags;
        asCreateInfo.maxGeometryCount = 1;
        asCreateInfo.pGeometryInfos   = &geometryCreate;

        // Create the acceleration structure object and allocate the memory
        // required to hold the TLAS data
        m_tlas.as = m_alloc->MakeAccelStruct(asCreateInfo);
        m_debug.setObjectName(m_tlas.as.handle, "Tlas");

        // Compute the amount of scratch memory required by the acceleration structure builder
        VkAccelerationStructureMemoryRequirementsInfoKHR memoryRequirementsInfo{
            VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_MEMORY_REQUIREMENTS_INFO_KHR};
        memoryRequirementsInfo.type =
            VK_ACCELERATION_STRUCTURE_MEMORY_REQUIREMENTS_TYPE_BUILD_SCRATCH_KHR;
        memoryRequirementsInfo.accelerationStructure = m_tlas.as.handle;
        memoryRequirementsInfo.buildType = VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR;

        VkMemoryRequirements2 reqMem{VK_STRUCTURE_TYPE_MEMORY_REQUIREMENTS_2};
        vkGetAccelerationStructureMemoryRequirementsKHR(m_device, &memoryRequirementsInfo, &reqMem);
        VkDeviceSize scratchSize = reqMem.memoryRequirements.size;


        // Allocate the scratch memory
        UniqueMemoryBuffer scratchBuffer =
            m_alloc->MakeBuffer(scratchSize, vk::BufferUsageFlagBits::eRayTracingKHR
                                                 | vk::BufferUsageFlagBits::eShaderDeviceAddress);
        VkBufferDeviceAddressInfo bufferInfo{VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO};
        bufferInfo.buffer              = scratchBuffer.handle;
        VkDeviceAddress scratchAddress = vkGetBufferDeviceAddress(m_device, &bufferInfo);


        // For each instance, build the corresponding instance descriptor
        std::vector<VkAccelerationStructureInstanceKHR> geometryInstances;
        geometryInstances.reserve(instances.size());
        for (const auto& inst : instances) {
            // A conversion is needed (for example) from glm::mat4 to the 3x4 row-major matrix
            // dictated by Vulkan API.
            geometryInstances.push_back(instanceToVkGeometryInstanceKHR(inst));
        }

        // Building the TLAS
        CommandPool       genCmdBuf(m_device, m_queueIndex);
        vk::CommandBuffer cmdBuf = genCmdBuf.MakeCmdBuffer();

        // Create a buffer holding the actual instance data for use by the AS
        // builder
        VkDeviceSize instanceDescsSizeInBytes =
            instances.size() * sizeof(VkAccelerationStructureInstanceKHR);

        // Allocate the instance buffer and copy its contents from host to device
        // memory
        m_instBuffer = m_alloc->MakeBuffer(cmdBuf, geometryInstances,
                                           vk::BufferUsageFlagBits::eRayTracingKHR
                                               | vk::BufferUsageFlagBits::eShaderDeviceAddress);
        m_debug.setObjectName(m_instBuffer.handle, "TLASInstances");
        // VkBufferDeviceAddressInfo bufferInfo{VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO};
        bufferInfo.buffer               = m_instBuffer.handle;
        VkDeviceAddress instanceAddress = vkGetBufferDeviceAddress(m_device, &bufferInfo);

        // Make sure the copy of the instance buffer are copied before triggering the
        // acceleration structure build
        VkMemoryBarrier barrier{VK_STRUCTURE_TYPE_MEMORY_BARRIER};
        barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
        barrier.dstAccessMask = VK_ACCESS_ACCELERATION_STRUCTURE_WRITE_BIT_KHR;
        vkCmdPipelineBarrier(cmdBuf, VK_PIPELINE_STAGE_TRANSFER_BIT,
                             VK_PIPELINE_STAGE_ACCELERATION_STRUCTURE_BUILD_BIT_KHR, 0, 1, &barrier,
                             0, nullptr, 0, nullptr);

        // Build the TLAS
        VkAccelerationStructureGeometryDataKHR geometry{
            VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_INSTANCES_DATA_KHR};
        geometry.instances.arrayOfPointers    = VK_FALSE;
        geometry.instances.data.deviceAddress = instanceAddress;
        VkAccelerationStructureGeometryKHR topASGeometry{
            VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR};
        topASGeometry.geometryType = VK_GEOMETRY_TYPE_INSTANCES_KHR;
        topASGeometry.geometry     = geometry;


        const VkAccelerationStructureGeometryKHR*   pGeometry = &topASGeometry;
        VkAccelerationStructureBuildGeometryInfoKHR topASInfo{
            VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR};
        topASInfo.flags                     = flags;
        topASInfo.update                    = VK_FALSE;
        topASInfo.srcAccelerationStructure  = VK_NULL_HANDLE;
        topASInfo.dstAccelerationStructure  = m_tlas.as.handle;
        topASInfo.geometryArrayOfPointers   = VK_FALSE;
        topASInfo.geometryCount             = 1;
        topASInfo.ppGeometries              = &pGeometry;
        topASInfo.scratchData.deviceAddress = scratchAddress;

        // Build Offsets info: n instances
        VkAccelerationStructureBuildOffsetInfoKHR buildOffsetInfo{
            static_cast<uint32_t>(instances.size()), 0, 0, 0};
        const VkAccelerationStructureBuildOffsetInfoKHR* pBuildOffsetInfo = &buildOffsetInfo;

        // Build the TLAS
        vkCmdBuildAccelerationStructureKHR(cmdBuf, 1, &topASInfo, &pBuildOffsetInfo);


        genCmdBuf.SubmitAndWait(cmdBuf);
        m_alloc->ReleaseAllStagingBuffers();
        scratchBuffer.DestroyFrom(m_device);
    }

    //--------------------------------------------------------------------------------------------------
    // Refit the TLAS using new instance matrices
    //
    // One could've use nvvk::RaytracingBuilder to update the matrices, but the update can be done
    // more efficiently if some of the buffer and memory references are at hand.
    void updateTlasMatrices(const std::vector<Instance>& instances)
    {
        VkDeviceSize bufferSize = instances.size() * sizeof(VkAccelerationStructureInstanceKHR);
        // Create a staging buffer on the host to upload the new instance data
        UniqueMemoryBuffer stagingBuffer =
            m_alloc->MakeBuffer(bufferSize, vk::BufferUsageFlagBits::eTransferSrc,
#if defined(NVVK_ALLOC_VMA)
                                VmaMemoryUsage::VMA_MEMORY_USAGE_CPU_TO_GPU
#else
                                vk::MemoryPropertyFlagBits::eHostCoherent
                                    | vk::MemoryPropertyFlagBits::eHostVisible
#endif
            );

        // Copy the instance data into the staging buffer
        auto* gInst = reinterpret_cast<VkAccelerationStructureInstanceKHR*>(
            m_device.mapMemory(stagingBuffer.memory, 0, VK_WHOLE_SIZE));
        for (int i = 0; i < instances.size(); i++) {
            gInst[i] = instanceToVkGeometryInstanceKHR(instances[i]);
        }
        m_device.unmapMemory(stagingBuffer.memory);

        // Compute the amount of scratch memory required by the AS builder to update
        VkAccelerationStructureMemoryRequirementsInfoKHR memoryRequirementsInfo{
            VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_MEMORY_REQUIREMENTS_INFO_KHR};
        memoryRequirementsInfo.type =
            VK_ACCELERATION_STRUCTURE_MEMORY_REQUIREMENTS_TYPE_UPDATE_SCRATCH_KHR;
        memoryRequirementsInfo.accelerationStructure = m_tlas.as.handle;
        memoryRequirementsInfo.buildType = VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR;

        VkMemoryRequirements2 reqMem{VK_STRUCTURE_TYPE_MEMORY_REQUIREMENTS_2};
        vkGetAccelerationStructureMemoryRequirementsKHR(m_device, &memoryRequirementsInfo, &reqMem);
        // If vk::BuildAccelerationStructureFlagBitsKHR::eAllowUpdate flag wasn't specified, the
        // scratch size returned by this request would be 0 and the following buffer creation would
        // fail.
        VkDeviceSize scratchSize = reqMem.memoryRequirements.size;

        // Allocate the scratch buffer
        UniqueMemoryBuffer scratchBuffer =
            m_alloc->MakeBuffer(scratchSize, vk::BufferUsageFlagBits::eRayTracingKHR
                                                 | vk::BufferUsageFlagBits::eShaderDeviceAddress);
        VkBufferDeviceAddressInfo bufferInfo{VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO};
        bufferInfo.buffer              = scratchBuffer.handle;
        VkDeviceAddress scratchAddress = vkGetBufferDeviceAddress(m_device, &bufferInfo);


        // Update the instance buffer on the device side and build the TLAS
        CommandPool       genCmdBuf(m_device, m_queueIndex);
        vk::CommandBuffer cmdBuf = genCmdBuf.MakeCmdBuffer();

        VkBufferCopy region{0, 0, bufferSize};
        vkCmdCopyBuffer(cmdBuf, stagingBuffer.handle, m_instBuffer.handle, 1, &region);

        // VkBufferDeviceAddressInfo bufferInfo{VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO};
        bufferInfo.buffer               = m_instBuffer.handle;
        VkDeviceAddress instanceAddress = vkGetBufferDeviceAddress(m_device, &bufferInfo);


        // Make sure the copy of the instance buffer are copied before triggering the
        // acceleration structure build
        VkMemoryBarrier barrier{VK_STRUCTURE_TYPE_MEMORY_BARRIER};
        barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
        barrier.dstAccessMask = VK_ACCESS_ACCELERATION_STRUCTURE_WRITE_BIT_KHR;
        vkCmdPipelineBarrier(cmdBuf, VK_PIPELINE_STAGE_TRANSFER_BIT,
                             VK_PIPELINE_STAGE_ACCELERATION_STRUCTURE_BUILD_BIT_KHR, 0, 1, &barrier,
                             0, nullptr, 0, nullptr);


        VkAccelerationStructureGeometryDataKHR geometry{
            VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_INSTANCES_DATA_KHR};
        geometry.instances.arrayOfPointers    = VK_FALSE;
        geometry.instances.data.deviceAddress = instanceAddress;
        VkAccelerationStructureGeometryKHR topASGeometry{
            VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR};
        topASGeometry.geometryType = VK_GEOMETRY_TYPE_INSTANCES_KHR;
        topASGeometry.geometry     = geometry;

        const VkAccelerationStructureGeometryKHR* pGeometry = &topASGeometry;


        VkAccelerationStructureBuildGeometryInfoKHR topASInfo{
            VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR};
        topASInfo.flags                    = m_tlas.flags;
        topASInfo.update                   = VK_TRUE;            // Updates the accel structure.
        topASInfo.srcAccelerationStructure = m_tlas.as.handle;   // Uses the existing TLAS as a
                                                                 // "base".
        topASInfo.dstAccelerationStructure  = m_tlas.as.handle;  // Writes to the same TLAS.
        topASInfo.geometryArrayOfPointers   = VK_FALSE;
        topASInfo.geometryCount             = 1;
        topASInfo.ppGeometries              = &pGeometry;
        topASInfo.scratchData.deviceAddress = scratchAddress;

        uint32_t                                         nbInstances = (uint32_t)instances.size();
        VkAccelerationStructureBuildOffsetInfoKHR        buildOffsetInfo  = {nbInstances, 0, 0, 0};
        const VkAccelerationStructureBuildOffsetInfoKHR* pBuildOffsetInfo = &buildOffsetInfo;

        // Build the TLAS

        // Update the acceleration structure. Note the VK_TRUE parameter to trigger the update,
        // and the existing TLAS being passed and updated in place
        vkCmdBuildAccelerationStructureKHR(cmdBuf, 1, &topASInfo, &pBuildOffsetInfo);

        genCmdBuf.SubmitAndWait(cmdBuf);

        scratchBuffer.DestroyFrom(m_device);
        stagingBuffer.DestroyFrom(m_device);
    }

    //--------------------------------------------------------------------------------------------------
    // Refit the BLAS from updated buffers
    //
    void updateBlas(uint32_t blasIdx)
    {
        Blas& blas = m_blas[blasIdx];

        // Compute the amount of scratch memory required by the AS builder to update    the BLAS
        VkAccelerationStructureMemoryRequirementsInfoKHR memoryRequirementsInfo{
            VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_MEMORY_REQUIREMENTS_INFO_KHR};
        memoryRequirementsInfo.type =
            VK_ACCELERATION_STRUCTURE_MEMORY_REQUIREMENTS_TYPE_UPDATE_SCRATCH_KHR;
        memoryRequirementsInfo.accelerationStructure = blas.as.handle;
        memoryRequirementsInfo.buildType = VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR;

        VkMemoryRequirements2 reqMem{VK_STRUCTURE_TYPE_MEMORY_REQUIREMENTS_2};
        vkGetAccelerationStructureMemoryRequirementsKHR(m_device, &memoryRequirementsInfo, &reqMem);
        VkDeviceSize scratchSize = reqMem.memoryRequirements.size;

        // Allocate the scratch buffer
        UniqueMemoryBuffer scratchBuffer =
            m_alloc->MakeBuffer(scratchSize, vk::BufferUsageFlagBits::eRayTracingKHR
                                                 | vk::BufferUsageFlagBits::eShaderDeviceAddress);
        VkBufferDeviceAddressInfo bufferInfo{VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO};
        bufferInfo.buffer              = scratchBuffer.handle;
        VkDeviceAddress scratchAddress = vkGetBufferDeviceAddress(m_device, &bufferInfo);


        const vk::AccelerationStructureGeometryKHR*   pGeometry = blas.asGeometry.data();
        vk::AccelerationStructureBuildGeometryInfoKHR asInfo;
        asInfo.type                      = vk::AccelerationStructureTypeKHR::eBottomLevel;
        asInfo.flags                     = blas.flags;
        asInfo.update                    = VK_TRUE;
        asInfo.srcAccelerationStructure  = blas.as.handle;
        asInfo.dstAccelerationStructure  = blas.as.handle;
        asInfo.geometryArrayOfPointers   = VK_FALSE;
        asInfo.geometryCount             = (uint32_t)blas.asGeometry.size();
        asInfo.ppGeometries              = &pGeometry;
        asInfo.scratchData.deviceAddress = scratchAddress;

        std::vector<const vk::AccelerationStructureBuildOffsetInfoKHR*> pBuildOffset(
            blas.asBuildOffsetInfo.size());
        for (size_t i = 0; i < blas.asBuildOffsetInfo.size(); i++)
            pBuildOffset[i] = &blas.asBuildOffsetInfo[i];

        // Update the instance buffer on the device side and build the TLAS
        CommandPool       genCmdBuf(m_device, m_queueIndex);
        vk::CommandBuffer cmdBuf = genCmdBuf.MakeCmdBuffer();


        // Update the acceleration structure. Note the VK_TRUE parameter to trigger the update,
        // and the existing BLAS being passed and updated in place
        cmdBuf.buildAccelerationStructureKHR(asInfo, pBuildOffset);

        genCmdBuf.SubmitAndWait(cmdBuf);
        scratchBuffer.DestroyFrom(m_device);
    }

  private:
    // Top-level acceleration structure
    struct Tlas {
        UniqueMemoryAccelStruct              as;
        VkAccelerationStructureCreateInfoKHR asInfo{
            VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR, nullptr, 0,
            VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR};
        VkBuildAccelerationStructureFlagsKHR flags;
    };

    //--------------------------------------------------------------------------------------------------
    // Vector containing all the BLASes built and referenced by the TLAS
    std::vector<Blas> m_blas;
    // Top-level acceleration structure
    Tlas m_tlas;
    // Instance buffer containing the matrices and BLAS ids
    UniqueMemoryBuffer m_instBuffer;

    vk::Device m_device = nullptr;
    uint32_t   m_queueIndex{0};

    UniqueMemoryAllocator* m_alloc = nullptr;
    nvvk::DebugUtil        m_debug;

#ifdef VULKAN_HPP
  public:
    void buildTlas(const std::vector<Instance>&           instances,
                   vk::BuildAccelerationStructureFlagsKHR flags)
    {
        buildTlas(instances, static_cast<VkBuildAccelerationStructureFlagsKHR>(flags));
    }

#endif
};

}  // namespace vkpbr
