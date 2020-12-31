#include "vk_raytrace.h"

#include "vk_utils.h"
#include "logging.h"

namespace vkpbr {

void RaytracingBuilderKHR::setup(const VkDevice& device, UniqueMemoryAllocator* allocator,
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

void RaytracingBuilderKHR::buildBlas(
    const std::vector<RaytracingBuilderKHR::BlasInput>& input,
    VkBuildAccelerationStructureFlagsKHR                flags)
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
    std::vector<VkAccelerationStructureBuildGeometryInfoKHR> buildInfos(nbBlas);
    for (uint32_t idx = 0; idx < nbBlas; idx++) {
        buildInfos[idx].sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR;
        buildInfos[idx].flags = flags;
        buildInfos[idx].geometryCount            = (uint32_t)m_blas[idx].input.asGeometry.size();
        buildInfos[idx].pGeometries              = m_blas[idx].input.asGeometry.data();
        buildInfos[idx].mode                     = VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR;
        buildInfos[idx].type                     = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
        buildInfos[idx].srcAccelerationStructure = VK_NULL_HANDLE;
    }

    // Finding sizes to create acceleration structures and scratch
    // Keep the largest scratch buffer size, to use only one scratch for all build
    VkDeviceSize              maxScratch{0};          // Largest scratch buffer for our BLAS
    std::vector<VkDeviceSize> originalSizes(nbBlas);  // use for stats

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
        VkAccelerationStructureBuildSizesInfoKHR sizeInfo{
            VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR};
        vkGetAccelerationStructureBuildSizesKHR(m_device,
                                                VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR,
                                                &buildInfos[idx], maxPrimCount.data(), &sizeInfo);

        // Create acceleration structure object. Not yet bound to memory.
        VkAccelerationStructureCreateInfoKHR createInfo{
            VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR};
        createInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
        createInfo.size = sizeInfo.accelerationStructureSize;  // Will be used to allocate
                                                               // memory.

        // Actual allocation of buffer and acceleration structure. Note: This relies on
        // createInfo.offset == 0 and fills in createInfo.buffer with the buffer allocated to
        // store the BLAS. The underlying vkCreateAccelerationStructureKHR call then consumes
        // the buffer value.
        m_blas[idx].as = m_alloc->MakeAccelStruct(createInfo);
        m_debug.setObjectName(m_blas[idx].as.handle,
                              (std::string("Blas" + std::to_string(idx)).c_str()));
        buildInfos[idx].dstAccelerationStructure = m_blas[idx].as.handle;  // Setting the where
                                                                          // the build lands

        // Keeping info
        m_blas[idx].flags = flags;
        maxScratch        = std::max(maxScratch, sizeInfo.buildScratchSize);

        // Stats - Original size
        originalSizes[idx] = sizeInfo.accelerationStructureSize;
    }

    // Allocate the scratch buffers holding the temporary data of the
    // acceleration structure builder
    UniqueMemoryBuffer scratchBuffer = m_alloc->MakeBuffer(
        maxScratch,  // VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT |
                     // VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        vk::BufferUsageFlagBits::eShaderDeviceAddress | vk::BufferUsageFlagBits::eStorageBuffer);
    VkBufferDeviceAddressInfo bufferInfo{VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO};
    bufferInfo.buffer              = scratchBuffer.handle;
    VkDeviceAddress scratchAddress = vkGetBufferDeviceAddress(m_device, &bufferInfo);


    // Is compaction requested?
    bool doCompaction = (flags & VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_COMPACTION_BIT_KHR)
                        == VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_COMPACTION_BIT_KHR;

    // Allocate a query pool for storing the needed size for every BLAS compaction.
    VkQueryPoolCreateInfo qpci{VK_STRUCTURE_TYPE_QUERY_POOL_CREATE_INFO};
    qpci.queryCount = nbBlas;
    qpci.queryType  = VK_QUERY_TYPE_ACCELERATION_STRUCTURE_COMPACTED_SIZE_KHR;
    VkQueryPool queryPool;
    vkCreateQueryPool(m_device, &qpci, nullptr, &queryPool);


    // Allocate a command pool for queue of given queue index.
    // To avoid timeout, record and submit one command buffer per AS build.
    vkpbr::CommandPool            genCmdBuf(m_device, m_queueIndex);
    std::vector<vk::CommandBuffer> allCmdBufs(nbBlas);

    // Building the acceleration structures
    for (uint32_t idx = 0; idx < nbBlas; idx++) {
        auto&           blas   = m_blas[idx];
        vk::CommandBuffer cmdBuf = genCmdBuf.MakeCmdBuffer();
        allCmdBufs[idx]        = cmdBuf;

        // All build are using the same scratch buffer
        buildInfos[idx].scratchData.deviceAddress = scratchAddress;

        // Convert user vector of offsets to vector of pointer-to-offset (required by vk).
        // Recall that this defines which (sub)section of the vertex/index arrays
        // will be built into the BLAS.
        std::vector<const VkAccelerationStructureBuildRangeInfoKHR*> pBuildOffset(
            blas.input.asBuildOffsetInfo.size());
        for (size_t infoIdx = 0; infoIdx < blas.input.asBuildOffsetInfo.size(); infoIdx++)
            pBuildOffset[infoIdx] = &blas.input.asBuildOffsetInfo[infoIdx];

        // Building the AS
        vkCmdBuildAccelerationStructuresKHR(cmdBuf, 1, &buildInfos[idx], pBuildOffset.data());

        // Since the scratch buffer is reused across builds, we need a barrier to ensure one
        // build is finished before starting the next one
        VkMemoryBarrier barrier{VK_STRUCTURE_TYPE_MEMORY_BARRIER};
        barrier.srcAccessMask = VK_ACCESS_ACCELERATION_STRUCTURE_WRITE_BIT_KHR;
        barrier.dstAccessMask = VK_ACCESS_ACCELERATION_STRUCTURE_READ_BIT_KHR;
        vkCmdPipelineBarrier(cmdBuf, VK_PIPELINE_STAGE_ACCELERATION_STRUCTURE_BUILD_BIT_KHR,
                             VK_PIPELINE_STAGE_ACCELERATION_STRUCTURE_BUILD_BIT_KHR, 0, 1, &barrier,
                             0, nullptr, 0, nullptr);

        // Write compacted size to query number idx.
        if (doCompaction) {
            //vkCmdWriteAccelerationStructuresPropertiesKHR(
            //    cmdBuf, 1, &blas.as.handle, VK_QUERY_TYPE_ACCELERATION_STRUCTURE_COMPACTED_SIZE_KHR,
            //    queryPool, idx);
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
        std::vector<VkDeviceSize> compactSizes(nbBlas);
        vkGetQueryPoolResults(m_device, queryPool, 0, (uint32_t)compactSizes.size(),
                              compactSizes.size() * sizeof(VkDeviceSize), compactSizes.data(),
                              sizeof(VkDeviceSize), VK_QUERY_RESULT_WAIT_BIT);


        // Compacting
        std::vector<UniqueMemoryAccelStruct> cleanupAS(nbBlas);  // previous AS to destroy
        uint32_t                    statTotalOriSize{0}, statTotalCompactSize{0};
        for (uint32_t idx = 0; idx < nbBlas; idx++) {
            // LOGI("Reducing %i, from %d to %d \n", i, originalSizes[i], compactSizes[i]);
            statTotalOriSize += (uint32_t)originalSizes[idx];
            statTotalCompactSize += (uint32_t)compactSizes[idx];

            // Creating a compact version of the AS
            VkAccelerationStructureCreateInfoKHR asCreateInfo{
                VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR};
            asCreateInfo.size = compactSizes[idx];
            asCreateInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
            auto as           = m_alloc->MakeAccelStruct(asCreateInfo);

            // Copy the original BLAS to a compact version
            VkCopyAccelerationStructureInfoKHR copyInfo{
                VK_STRUCTURE_TYPE_COPY_ACCELERATION_STRUCTURE_INFO_KHR};
            copyInfo.src  = m_blas[idx].as.handle;
            copyInfo.dst  = as.handle;
            copyInfo.mode = VK_COPY_ACCELERATION_STRUCTURE_MODE_COMPACT_KHR;
            vkCmdCopyAccelerationStructureKHR(cmdBuf, &copyInfo);
            cleanupAS[idx] = m_blas[idx].as;
            m_blas[idx].as = as;
        }
        genCmdBuf.SubmitAndWait(cmdBuf);  // vkQueueWaitIdle within.

        // Destroying the previous version
        for (auto as : cleanupAS)
            as.DestroyFrom(m_device);

        LOG(INFO) << Format(" RT BLAS: reducing from: %u to: %u = %u (%2.2f%s smaller) \n", statTotalOriSize,
             statTotalCompactSize, statTotalOriSize - statTotalCompactSize,
             (statTotalOriSize - statTotalCompactSize) / float(statTotalOriSize) * 100.f, "%%");
    }

    vkDestroyQueryPool(m_device, queryPool, nullptr);
    scratchBuffer.DestroyFrom(m_device);
    m_alloc->ReleaseAllStagingBuffers();
}

VkAccelerationStructureInstanceKHR RaytracingBuilderKHR::instanceToVkGeometryInstanceKHR(
    const Instance& instance)
{
    assert(size_t(instance.blasId) < m_blas.size());
    BlasEntry& blas{m_blas[instance.blasId]};

    VkAccelerationStructureDeviceAddressInfoKHR addressInfo{
        VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_DEVICE_ADDRESS_INFO_KHR};
    addressInfo.accelerationStructure = blas.as.handle;
    VkDeviceAddress blasAddress =
        vkGetAccelerationStructureDeviceAddressKHR(m_device, &addressInfo);

    VkAccelerationStructureInstanceKHR gInst{};
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
    gInst.flags                                  = instance.flags;
    gInst.accelerationStructureReference         = blasAddress;
    return gInst;
}

//--------------------------------------------------------------------------------------------------
// Creating the top-level acceleration structure from the vector of Instance
// - See struct of Instance
// - The resulting TLAS will be stored in m_tlas
// - update is to rebuild the Tlas with updated matrices

void RaytracingBuilderKHR::buildTlas(const std::vector<Instance>&         instances,
                                            VkBuildAccelerationStructureFlagsKHR flags, bool update)
{
    // Cannot call buildTlas twice except to update.
    if (!update)
        assert(!m_tlas.as.handle);

    vkpbr::CommandPool genCmdBuf(m_device, m_queueIndex);
    vk::CommandBuffer   cmdBuf = genCmdBuf.MakeCmdBuffer();

    m_tlas.flags = flags;

    // Convert array of our Instances to an array native Vulkan instances.
    std::vector<VkAccelerationStructureInstanceKHR> geometryInstances;
    geometryInstances.reserve(instances.size());
    for (const auto& inst : instances) {
        geometryInstances.push_back(instanceToVkGeometryInstanceKHR(inst));
    }

    // Create a buffer holding the actual instance data (matrices++) for use by the AS builder
    VkDeviceSize instanceDescsSizeInBytes =
        instances.size() * sizeof(VkAccelerationStructureInstanceKHR);

    // Allocate the instance buffer and copy its contents from host to device memory
    if (update)
        m_instBuffer.DestroyFrom(m_device);
    m_instBuffer =
        m_alloc->MakeBuffer(cmdBuf, geometryInstances, vk::BufferUsageFlagBits::eShaderDeviceAddress);
    m_debug.setObjectName(m_instBuffer.handle, "TLASInstances");
    VkBufferDeviceAddressInfo bufferInfo{VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO};
    bufferInfo.buffer               = m_instBuffer.handle;
    VkDeviceAddress instanceAddress = vkGetBufferDeviceAddress(m_device, &bufferInfo);

    // Make sure the copy of the instance buffer are copied before triggering the
    // acceleration structure build
    VkMemoryBarrier barrier{VK_STRUCTURE_TYPE_MEMORY_BARRIER};
    barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
    barrier.dstAccessMask = VK_ACCESS_ACCELERATION_STRUCTURE_WRITE_BIT_KHR;
    vkCmdPipelineBarrier(cmdBuf, VK_PIPELINE_STAGE_TRANSFER_BIT,
                         VK_PIPELINE_STAGE_ACCELERATION_STRUCTURE_BUILD_BIT_KHR, 0, 1, &barrier, 0,
                         nullptr, 0, nullptr);


    //--------------------------------------------------------------------------------------------------

    // Create VkAccelerationStructureGeometryInstancesDataKHR
    // This wraps a device pointer to the above uploaded instances.
    VkAccelerationStructureGeometryInstancesDataKHR instancesVk{
        VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_INSTANCES_DATA_KHR};
    instancesVk.arrayOfPointers    = VK_FALSE;
    instancesVk.data.deviceAddress = instanceAddress;

    // Put the above into a VkAccelerationStructureGeometryKHR. We need to put the
    // instances struct in a union and label it as instance data.
    VkAccelerationStructureGeometryKHR topASGeometry{
        VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR};
    topASGeometry.geometryType       = VK_GEOMETRY_TYPE_INSTANCES_KHR;
    topASGeometry.geometry.instances = instancesVk;

    // Find sizes
    VkAccelerationStructureBuildGeometryInfoKHR buildInfo{
        VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR};
    buildInfo.flags         = flags;
    buildInfo.geometryCount = 1;
    buildInfo.pGeometries   = &topASGeometry;
    buildInfo.mode          = update ? VK_BUILD_ACCELERATION_STRUCTURE_MODE_UPDATE_KHR :
                              VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR;
    buildInfo.type                     = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;
    buildInfo.srcAccelerationStructure = VK_NULL_HANDLE;

    uint32_t                                 count = (uint32_t)instances.size();
    VkAccelerationStructureBuildSizesInfoKHR sizeInfo{
        VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR};
    vkGetAccelerationStructureBuildSizesKHR(
        m_device, VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR, &buildInfo, &count, &sizeInfo);


    // Create TLAS
    if (update == false) {
        VkAccelerationStructureCreateInfoKHR createInfo{
            VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR};
        createInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;
        createInfo.size = sizeInfo.accelerationStructureSize;

        m_tlas.as = m_alloc->MakeAccelStruct(createInfo);
        m_debug.setObjectName(m_tlas.as.handle, "Tlas");
    }

    // Allocate the scratch memory
    UniqueMemoryBuffer scratchBuffer =
        m_alloc->MakeBuffer(sizeInfo.buildScratchSize,
                            vk::BufferUsageFlagBits::eAccelerationStructureStorageKHR
                                | vk::BufferUsageFlagBits::eShaderDeviceAddress);
    // VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_STORAGE_BIT_KHR   |
    // VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT);
    bufferInfo.buffer              = scratchBuffer.handle;
    VkDeviceAddress scratchAddress = vkGetBufferDeviceAddress(m_device, &bufferInfo);


    // Update build information
    buildInfo.srcAccelerationStructure  = update ? m_tlas.as.handle : nullptr;
    buildInfo.dstAccelerationStructure  = m_tlas.as.handle;
    buildInfo.scratchData.deviceAddress = scratchAddress;


    // Build Offsets info: n instances
    VkAccelerationStructureBuildRangeInfoKHR buildOffsetInfo{
        static_cast<uint32_t>(instances.size()), 0, 0, 0};
    const VkAccelerationStructureBuildRangeInfoKHR* pBuildOffsetInfo = &buildOffsetInfo;

    // Build the TLAS
    vkCmdBuildAccelerationStructuresKHR(cmdBuf, 1, &buildInfo, &pBuildOffsetInfo);

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
    VkAccelerationStructureBuildGeometryInfoKHR buildInfos{
        VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR};
    buildInfos.flags         = blas.flags;
    buildInfos.geometryCount = (uint32_t)blas.input.asGeometry.size();
    buildInfos.pGeometries   = blas.input.asGeometry.data();
    buildInfos.mode          = VK_BUILD_ACCELERATION_STRUCTURE_MODE_UPDATE_KHR;  // UPDATE
    buildInfos.type          = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
    buildInfos.srcAccelerationStructure = blas.as.handle;  // UPDATE
    buildInfos.dstAccelerationStructure = blas.as.handle;

    // Find size to build on the device
    std::vector<uint32_t> maxPrimCount(blas.input.asBuildOffsetInfo.size());
    for (auto tt = 0; tt < blas.input.asBuildOffsetInfo.size(); tt++)
        maxPrimCount[tt] = blas.input.asBuildOffsetInfo[tt].primitiveCount;  // Number of
                                                                             // primitives/triangles
    VkAccelerationStructureBuildSizesInfoKHR sizeInfo{
        VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR};
    vkGetAccelerationStructureBuildSizesKHR(m_device,
                                            VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR,
                                            &buildInfos, maxPrimCount.data(), &sizeInfo);

    // Allocate the scratch buffer and setting the scratch info
    UniqueMemoryBuffer scratchBuffer =
        m_alloc->MakeBuffer(sizeInfo.buildScratchSize,
                            vk::BufferUsageFlagBits::eAccelerationStructureStorageKHR
                                | vk::BufferUsageFlagBits::eShaderDeviceAddress);
    // VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_STORAGE_BIT_KHR
    //    | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT);
    VkBufferDeviceAddressInfo bufferInfo{VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO};
    bufferInfo.buffer                    = scratchBuffer.handle;
    buildInfos.scratchData.deviceAddress = vkGetBufferDeviceAddress(m_device, &bufferInfo);


    std::vector<const VkAccelerationStructureBuildRangeInfoKHR*> pBuildOffset(
        blas.input.asBuildOffsetInfo.size());
    for (size_t i = 0; i < blas.input.asBuildOffsetInfo.size(); i++)
        pBuildOffset[i] = &blas.input.asBuildOffsetInfo[i];

    // Update the instance buffer on the device side and build the TLAS
    vkpbr::CommandPool genCmdBuf(m_device, m_queueIndex);
    vk::CommandBuffer   cmdBuf = genCmdBuf.MakeCmdBuffer();


    // Update the acceleration structure. Note the VK_TRUE parameter to trigger the update,
    // and the existing BLAS being passed and updated in place
    vkCmdBuildAccelerationStructuresKHR(cmdBuf, 1, &buildInfos, pBuildOffset.data());

    genCmdBuf.SubmitAndWait(cmdBuf);
    scratchBuffer.DestroyFrom(m_device);
}

}  // namespace vkpbr
