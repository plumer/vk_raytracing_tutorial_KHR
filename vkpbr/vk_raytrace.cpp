#include "vk_raytrace.h"

#include "logging.h"
#include "vk_utils.h"

namespace vkpbr {

void RaytracingBuilderKHR::setup(const vk::Device& device, UniqueMemoryAllocator* allocator,
                                 uint32_t queueIndex)
{
    m_device     = device;
    m_queueIndex = queueIndex;
    m_debug.setup(device);
    m_alloc = allocator;
}

void RaytracingBuilderKHR::destroy()
{
    for (auto& b : m_blas) {
        b.as.DestroyFrom(m_device);
    }
    m_tlas.as.DestroyFrom(m_device);
    m_instBuffer.DestroyFrom(m_device);
    m_blas.clear();
    m_tlas = {};
}

void RaytracingBuilderKHR::buildBlas(const std::vector<RaytracingBuilderKHR::BlasInput>& input,
                                     vk::BuildAccelerationStructureFlagsKHR              flags)
{
    // Cannot call buildBlas twice.
    assert(m_blas.empty());

    // Make our own copy of the user-provided inputs.
    m_blas          = std::vector<BlasEntry>(input.begin(), input.end());
    uint32_t nbBlas = static_cast<uint32_t>(m_blas.size());

    // Preparing the build information array for the acceleration build command.
    // This is mostly just a fancy pointer to the user-passed arrays of
    // VkAccelerationStructureGeometryKHR. dstAccelerationStructure will be filled later once we
    // allocated the acceleration structures.
    std::vector<vk::AccelerationStructureBuildGeometryInfoKHR> buildInfos(nbBlas);
    for (uint32_t idx = 0; idx < nbBlas; idx++) {
        buildInfos[idx].flags                    = flags;
        buildInfos[idx].setGeometries(m_blas[idx].input.asGeometry);
        //= (uint32_t)m_blas[idx].input.asGeometry.size();
        //buildInfos[idx].pGeometries              = m_blas[idx].input.asGeometry.data();
        buildInfos[idx].mode                     = vk::BuildAccelerationStructureModeKHR::eBuild;
        buildInfos[idx].type                     = vk::AccelerationStructureTypeKHR::eBottomLevel;
        buildInfos[idx].srcAccelerationStructure = nullptr;
    }

    // Finding sizes to create acceleration structures and scratch
    // Keep the largest scratch buffer size, to use only one scratch for all build
    vk::DeviceSize              maxScratch{0};          // Largest scratch buffer for our BLAS
    std::vector<vk::DeviceSize> originalSizes(nbBlas);  // use for stats

    for (size_t idx = 0; idx < nbBlas; idx++) {
        // Query both the size of the finished acceleration structure and the  amount of scratch
        // memory needed (both written to sizeInfo). The
        // `vkGetAccelerationStructureBuildSizesKHR` function computes the worst case memory
        // requirements based on the user-reported max number of primitives. Later, compaction
        // can fix this potential inefficiency.
        std::vector<uint32_t> maxPrimCount(m_blas[idx].input.asBuildOffsetInfo.size());
        for (auto tt = 0; tt < m_blas[idx].input.asBuildOffsetInfo.size(); tt++)
            maxPrimCount[tt] =
                m_blas[idx].input.asBuildOffsetInfo[tt].primitiveCount;  // Number of
                                                                         // primitives/triangles
        std::vector<u32> max_prim_count;
        for (const auto& build_offset_info : m_blas[idx].input.asBuildOffsetInfo) {
            max_prim_count.push_back(build_offset_info.primitiveCount);
        }
        assert(max_prim_count == maxPrimCount);

        auto size_info = m_device.getAccelerationStructureBuildSizesKHR(
            vk::AccelerationStructureBuildTypeKHR::eDevice, buildInfos[idx], maxPrimCount);

        // Create acceleration structure object. Not yet bound to memory.
        auto accel_struct_ci = vk::AccelerationStructureCreateInfoKHR()
                                   .setType(vk::AccelerationStructureTypeKHR::eBottomLevel)
                                   // Used to allocate memory.
                                   .setSize(size_info.accelerationStructureSize);
        // Actual allocation of buffer and acceleration structure. Note: This relies on
        // createInfo.offset == 0 and fills in createInfo.buffer with the buffer allocated to
        // store the BLAS. The underlying vkCreateAccelerationStructureKHR call then consumes
        // the buffer value.
        m_blas[idx].as = m_alloc->MakeAccelStruct(accel_struct_ci);

        m_debug.setObjectName(m_blas[idx].as.handle,
                              (std::string("Blas" + std::to_string(idx)).c_str()));
        buildInfos[idx].dstAccelerationStructure = m_blas[idx].as.handle;  // Setting the where
                                                                           // the build lands

        // Keeping info
        m_blas[idx].flags = flags;
        maxScratch        = std::max(maxScratch, size_info.buildScratchSize);

        // Stats - Original size
        originalSizes[idx] = size_info.accelerationStructureSize;
    }

    // Allocate the scratch buffers holding the temporary data of the
    // acceleration structure builder
    UniqueMemoryBuffer scratchBuffer =
        m_alloc->MakeBuffer(maxScratch, vk::BufferUsageFlagBits::eShaderDeviceAddress
                                            | vk::BufferUsageFlagBits::eStorageBuffer);
    vk::DeviceAddress scratchAddress = m_device.getBufferAddress({scratchBuffer.handle});

    // Is compaction requested?
    bool doCompaction =
        FlagsMatch(flags, vk::BuildAccelerationStructureFlagBitsKHR::eAllowCompaction);

    // Allocate a query pool for storing the needed size for every BLAS compaction.
    vk::QueryPoolCreateInfo qpci;
    qpci.queryCount         = nbBlas;
    qpci.queryType          = vk::QueryType::eAccelerationStructureCompactedSizeKHR;
    vk::QueryPool queryPool = m_device.createQueryPool(qpci);


    // Allocate a command pool for queue of given queue index.
    // To avoid timeout, record and submit one command buffer per AS build.
    vkpbr::CommandPool             genCmdBuf(m_device, m_queueIndex);
    std::vector<vk::CommandBuffer> allCmdBufs(nbBlas);

    // Building the acceleration structures
    for (uint32_t idx = 0; idx < nbBlas; idx++) {
        auto&             blas   = m_blas[idx];
        vk::CommandBuffer cmdBuf = genCmdBuf.MakeCmdBuffer();
        allCmdBufs[idx]          = cmdBuf;

        // All build are using the same scratch buffer
        buildInfos[idx].scratchData.deviceAddress = scratchAddress;

        // Convert user vector of offsets to vector of pointer-to-offset (required by vk).
        // Recall that this defines which (sub)section of the vertex/index arrays
        // will be built into the BLAS.
        std::vector<const vk::AccelerationStructureBuildRangeInfoKHR*> p_build_offset;
        for (const auto& build_offset_info : blas.input.asBuildOffsetInfo) {
            p_build_offset.push_back(&build_offset_info);
        }
        // Building the AS
        cmdBuf.buildAccelerationStructuresKHR(buildInfos[idx], p_build_offset);

        // Since the scratch buffer is reused across builds, we need a barrier to ensure one
        // build is finished before starting the next one
        auto mem_barrier = vk::MemoryBarrier()
                               .setSrcAccessMask(vk::AccessFlagBits::eAccelerationStructureWriteKHR)
                               .setDstAccessMask(vk::AccessFlagBits::eAccelerationStructureReadKHR);
        cmdBuf.pipelineBarrier(vk::PipelineStageFlagBits::eAccelerationStructureBuildKHR,
                               vk::PipelineStageFlagBits::eAccelerationStructureBuildKHR, {},
                               mem_barrier, {}, {});

        // Write compacted size to query number idx.
        if (doCompaction) {
            cmdBuf.writeAccelerationStructuresPropertiesKHR(
                blas.as.handle, vk::QueryType::eAccelerationStructureCompactedSizeKHR, queryPool,
                idx);
        }
    }
    genCmdBuf.SubmitAndWait(allCmdBufs);  // vkQueueWaitIdle behind this call.
    allCmdBufs.clear();

    // Compacting all BLAS
    if (doCompaction) {
        vk::CommandBuffer cmdBuf = genCmdBuf.MakeCmdBuffer();

        // Get the size result back
        std::vector<vk::DeviceSize> compactSizes(nbBlas);
        vkGetQueryPoolResults(m_device, queryPool, 0, (uint32_t)compactSizes.size(),
                              compactSizes.size() * sizeof(vk::DeviceSize), compactSizes.data(),
                              sizeof(vk::DeviceSize), VK_QUERY_RESULT_WAIT_BIT);

        // Compacting
        std::vector<UniqueMemoryAccelStruct> cleanupAS(nbBlas);  // previous AS to destroy
        uint32_t                             statTotalOriSize{0}, statTotalCompactSize{0};
        for (uint32_t idx = 0; idx < nbBlas; idx++) {
            // LOGI("Reducing %i, from %d to %d \n", i, originalSizes[i], compactSizes[i]);
            statTotalOriSize += (uint32_t)originalSizes[idx];
            statTotalCompactSize += (uint32_t)compactSizes[idx];

            // Creating a compact version of the AS
            vk::AccelerationStructureCreateInfoKHR asCreateInfo;
            asCreateInfo.size = compactSizes[idx];
            asCreateInfo.type = vk::AccelerationStructureTypeKHR::eBottomLevel;
            auto as           = m_alloc->MakeAccelStruct(asCreateInfo);

            // Copy the original BLAS to a compact version
            vk::CopyAccelerationStructureInfoKHR copyInfo;
            copyInfo.src  = m_blas[idx].as.handle;
            copyInfo.dst  = as.handle;
            copyInfo.mode = vk::CopyAccelerationStructureModeKHR::eCompact;
            cmdBuf.copyAccelerationStructureKHR(copyInfo);
            cleanupAS[idx] = m_blas[idx].as;
            m_blas[idx].as = as;
        }
        genCmdBuf.SubmitAndWait(cmdBuf);  // vkQueueWaitIdle within.

        // Destroying the previous version
        for (auto as : cleanupAS)
            as.DestroyFrom(m_device);

        LOG(INFO) << Format(
            " RT BLAS: reducing from: %u to: %u = %u (%2.2f%s smaller) \n", statTotalOriSize,
            statTotalCompactSize, statTotalOriSize - statTotalCompactSize,
            (statTotalOriSize - statTotalCompactSize) / float(statTotalOriSize) * 100.f, "%%");
    }

    //vkDestroyQueryPool(m_device, queryPool, nullptr);
    m_device.destroyQueryPool(queryPool);
    scratchBuffer.DestroyFrom(m_device);
    m_alloc->ReleaseAllStagingBuffers();
}

vk::AccelerationStructureInstanceKHR RaytracingBuilderKHR::instanceToVkGeometryInstanceKHR(
    const Instance& instance)
{
    assert(size_t(instance.blasId) < m_blas.size());
    BlasEntry& blas{m_blas[instance.blasId]};

    vk::DeviceAddress blas_addr = m_device.getAccelerationStructureAddressKHR({blas.as.handle});

    // The matrices for the instance transforms are row-major, instead of
    // column-major in the rest of the application
    glm::mat4 transform_t = glm::transpose(instance.transform);

    auto gInst = vk::AccelerationStructureInstanceKHR()
                     .setInstanceCustomIndex(instance.instanceId)
                     .setMask(instance.mask)
                     .setInstanceShaderBindingTableRecordOffset(instance.hitGroupId)
                     .setFlags(instance.flags)
                     .setAccelerationStructureReference(blas_addr);
    // The gInst.transform value only contains 12 values, corresponding to a 4x3 matrix, hence
    // saving the last row that is anyway always (0,0,0,1). Since the matrix is row-major, we simply
    // copy the first 12 values of the original 4x4 matrix.
    memcpy(&gInst.transform, &transform_t, sizeof(gInst.transform));

    return gInst;
}

//--------------------------------------------------------------------------------------------------
// Creating the top-level acceleration structure from the vector of Instance
// - See struct of Instance
// - The resulting TLAS will be stored in m_tlas
// - update is to rebuild the Tlas with updated matrices

void RaytracingBuilderKHR::buildTlas(const std::vector<Instance>&           instances,
                                     vk::BuildAccelerationStructureFlagsKHR flags, bool update)
{
    // Cannot call buildTlas twice except to update.
    if (!update)
        assert(!m_tlas.as.handle);

    vkpbr::CommandPool genCmdBuf(m_device, m_queueIndex);
    vk::CommandBuffer  cmdBuf = genCmdBuf.MakeCmdBuffer();

    m_tlas.flags = flags;

    // Convert array of our Instances to an array native Vulkan instances.
    std::vector<vk::AccelerationStructureInstanceKHR> geometryInstances;
    geometryInstances.reserve(instances.size());
    for (const auto& inst : instances) {
        geometryInstances.push_back(instanceToVkGeometryInstanceKHR(inst));
    }

    // Create a buffer holding the actual instance data (matrices++) for use by the AS builder
    vk::DeviceSize instanceDescsSizeInBytes =
        instances.size() * sizeof(vk::AccelerationStructureInstanceKHR);

    // Allocate the instance buffer and copy its contents from host to device memory
    if (update)
        m_instBuffer.DestroyFrom(m_device);
    m_instBuffer = m_alloc->MakeBuffer(cmdBuf, geometryInstances,
                                       vk::BufferUsageFlagBits::eShaderDeviceAddress);
    m_debug.setObjectName(m_instBuffer.handle, "TLASInstances");
    vk::DeviceAddress instanceAddress = m_device.getBufferAddress({m_instBuffer.handle});

    // Make sure the copy of the instance buffer are copied before triggering the
    // acceleration structure build
    vk::MemoryBarrier barrier;
    barrier.srcAccessMask = vk::AccessFlagBits::eTransferWrite;
    barrier.dstAccessMask = vk::AccessFlagBits::eAccelerationStructureWriteKHR;
    cmdBuf.pipelineBarrier(vk::PipelineStageFlagBits::eTransfer,
                           vk::PipelineStageFlagBits::eAccelerationStructureBuildKHR, {}, barrier,
                           {}, {});

    //--------------------------------------------------------------------------------------------------

    // Create VkAccelerationStructureGeometryInstancesDataKHR
    // This wraps a device pointer to the above uploaded instances.
    vk::AccelerationStructureGeometryInstancesDataKHR instancesVk;
    instancesVk.arrayOfPointers    = VK_FALSE;
    instancesVk.data.deviceAddress = instanceAddress;

    // Put the above into a vk::AccelerationStructureGeometryKHR. We need to put the
    // instances struct in a union and label it as instance data.
    vk::AccelerationStructureGeometryKHR topASGeometry;
    topASGeometry.geometryType       = vk::GeometryTypeKHR::eInstances;
    topASGeometry.geometry.instances = instancesVk;

    // Find sizes
    vk::AccelerationStructureBuildGeometryInfoKHR buildInfo;
    buildInfo.flags         = flags;
    buildInfo.geometryCount = 1;
    buildInfo.pGeometries   = &topASGeometry;
    buildInfo.mode          = update ? vk::BuildAccelerationStructureModeKHR::eUpdate :
                              vk::BuildAccelerationStructureModeKHR::eBuild;
    buildInfo.type                     = vk::AccelerationStructureTypeKHR::eTopLevel;
    buildInfo.srcAccelerationStructure = nullptr;

    uint32_t count    = (uint32_t)instances.size();
    auto     sizeInfo = m_device.getAccelerationStructureBuildSizesKHR(
        vk::AccelerationStructureBuildTypeKHR::eDevice, buildInfo, count);


    // Create TLAS
    if (update == false) {
        vk::AccelerationStructureCreateInfoKHR createInfo;
        createInfo.type = vk::AccelerationStructureTypeKHR::eTopLevel;
        createInfo.size = sizeInfo.accelerationStructureSize;

        m_tlas.as = m_alloc->MakeAccelStruct(createInfo);
        m_debug.setObjectName(m_tlas.as.handle, "Tlas");
    }

    // Allocate the scratch memory
    UniqueMemoryBuffer scratchBuffer =
        m_alloc->MakeBuffer(sizeInfo.buildScratchSize,
                            vk::BufferUsageFlagBits::eAccelerationStructureStorageKHR
                                | vk::BufferUsageFlagBits::eShaderDeviceAddress);
    vk::DeviceAddress scratchAddress = m_device.getBufferAddress({scratchBuffer.handle});

    // Update build information
    buildInfo.srcAccelerationStructure  = update ? m_tlas.as.handle : nullptr;
    buildInfo.dstAccelerationStructure  = m_tlas.as.handle;
    buildInfo.scratchData.deviceAddress = scratchAddress;

    // Build Offsets info: n instances
    vk::AccelerationStructureBuildRangeInfoKHR buildOffsetInfo{cast_u32(instances.size()), 0, 0, 0};
    const vk::AccelerationStructureBuildRangeInfoKHR* pBuildOffsetInfo = &buildOffsetInfo;

    // Build the TLAS
    cmdBuf.buildAccelerationStructuresKHR(buildInfo, pBuildOffsetInfo);

    genCmdBuf.SubmitAndWait(cmdBuf);  // queueWaitIdle inside.
    m_alloc->ReleaseAllStagingBuffers();
    scratchBuffer.DestroyFrom(m_device);
}

//--------------------------------------------------------------------------------------------------
// Refit BLAS number blasIdx from updated buffer contents.
//

void RaytracingBuilderKHR::updateBlas(uint32_t blasIdx)
{
    assert(size_t(blasIdx) < m_blas.size());
    BlasEntry& blas = m_blas[blasIdx];  // The blas to update

    // Preparing all build information, acceleration is filled later
    auto buildInfos = vk::AccelerationStructureBuildGeometryInfoKHR()
                          .setFlags(vk::BuildAccelerationStructureFlagsKHR(blas.flags))
                          .setGeometries(blas.input.asGeometry)
                          .setMode(vk::BuildAccelerationStructureModeKHR::eUpdate)
                          .setType(vk::AccelerationStructureTypeKHR::eBottomLevel)
                          // Sets src and dst to be the same, for updating.
                          .setSrcAccelerationStructure(blas.as.handle)
                          .setDstAccelerationStructure(blas.as.handle);

    // Find size to build on the device
    std::vector<uint32_t> maxPrimCount(blas.input.asBuildOffsetInfo.size());
    for (auto tt = 0; tt < blas.input.asBuildOffsetInfo.size(); tt++)
        maxPrimCount[tt] = blas.input.asBuildOffsetInfo[tt].primitiveCount;  // Number of
                                                                             // primitives/triangles

    auto sizeInfo = m_device.getAccelerationStructureBuildSizesKHR(
        vk::AccelerationStructureBuildTypeKHR::eDevice, buildInfos, maxPrimCount);

    // Allocate the scratch buffer and setting the scratch info
    UniqueMemoryBuffer scratchBuffer =
        m_alloc->MakeBuffer(sizeInfo.buildScratchSize,
                            vk::BufferUsageFlagBits::eAccelerationStructureStorageKHR
                                | vk::BufferUsageFlagBits::eShaderDeviceAddress);

    buildInfos.scratchData.deviceAddress = m_device.getBufferAddress({scratchBuffer.handle});

    std::vector<const vk::AccelerationStructureBuildRangeInfoKHR*> pBuildOffset(
        blas.input.asBuildOffsetInfo.size());
    for (size_t i = 0; i < blas.input.asBuildOffsetInfo.size(); i++)
        pBuildOffset[i] = &blas.input.asBuildOffsetInfo[i];

    // Update the instance buffer on the device side and build the TLAS
    vkpbr::CommandPool genCmdBuf(m_device, m_queueIndex);
    vk::CommandBuffer  cmdBuf = genCmdBuf.MakeCmdBuffer();

    // Update the acceleration structure. Note the VK_TRUE parameter to trigger the update,
    // and the existing BLAS being passed and updated in place
    cmdBuf.buildAccelerationStructuresKHR(buildInfos, pBuildOffset);

    genCmdBuf.SubmitAndWait(cmdBuf);
    scratchBuffer.DestroyFrom(m_device);
}

}  // namespace vkpbr
