#include "vk_utils.h"

#include "nvvk/commands_vk.hpp"

#include "logging.h"

namespace {
using Binding   = vk::DescriptorSetLayoutBinding;
using Type      = vk::DescriptorType;
using Stage     = vk::ShaderStageFlags;
using StageBits = vk::ShaderStageFlagBits;
using Writer    = vk::WriteDescriptorSet;

vk::AccessFlags AccessFlagsForImageLayout(vk::ImageLayout layout)
{
    switch (layout) {
        case vk::ImageLayout::ePreinitialized:
            return vk::AccessFlagBits::eHostWrite;
        case vk::ImageLayout::eTransferDstOptimal:
            return vk::AccessFlagBits::eTransferWrite;
        case vk::ImageLayout::eTransferSrcOptimal:
            return vk::AccessFlagBits::eTransferRead;
        case vk::ImageLayout::eColorAttachmentOptimal:
            return vk::AccessFlagBits::eColorAttachmentWrite;
        case vk::ImageLayout::eDepthStencilAttachmentOptimal:
            return vk::AccessFlagBits::eDepthStencilAttachmentWrite;
        case vk::ImageLayout::eShaderReadOnlyOptimal:
            return vk::AccessFlagBits::eShaderRead;
        default:
            return vk::AccessFlags();
    }
}

vk::PipelineStageFlags PipelineStageForLayout(vk::ImageLayout layout)
{
    switch (layout) {
        case vk::ImageLayout::eTransferDstOptimal:
        case vk::ImageLayout::eTransferSrcOptimal:
            return vk::PipelineStageFlagBits::eTransfer;
        case vk::ImageLayout::eColorAttachmentOptimal:
            return vk::PipelineStageFlagBits::eColorAttachmentOutput;
        case vk::ImageLayout::eDepthStencilAttachmentOptimal:
            return vk::PipelineStageFlagBits::eEarlyFragmentTests;
        case vk::ImageLayout::eShaderReadOnlyOptimal:
            return vk::PipelineStageFlagBits::eFragmentShader;
        case vk::ImageLayout::ePreinitialized:
            return vk::PipelineStageFlagBits::eHost;
        case vk::ImageLayout::eUndefined:
            return vk::PipelineStageFlagBits::eTopOfPipe;
        default:
            return vk::PipelineStageFlagBits::eBottomOfPipe;
    }
}
}  // namespace

namespace vkpbr {

CommandPool::CommandPool(const vk::Device& device, u32 queue_family_index,
                         vk::CommandPoolCreateFlags flags, vk::Queue default_queue)
    : device_(device)
    , default_queue_(default_queue ? default_queue : device.getQueue(queue_family_index, 0))
{
    CHECK(device);

    auto cmd_pool_ci =
        vk::CommandPoolCreateInfo().setFlags(flags).setQueueFamilyIndex(queue_family_index);
    cmd_pool_ = device_.createCommandPool(cmd_pool_ci);
}
CommandPool::~CommandPool()
{
    if (cmd_pool_) {
        device_.destroyCommandPool(cmd_pool_);
    }
}

vk::CommandBuffer CommandPool::MakeCmdBuffer(
    vk::CommandBufferLevel level, bool begin, vk::CommandBufferUsageFlags usage,
    const vk::CommandBufferInheritanceInfo* p_inheritance_info)
{
    auto cmd_alloc_info = vk::CommandBufferAllocateInfo()
                              .setLevel(level)
                              .setCommandPool(cmd_pool_)
                              .setCommandBufferCount(1);

    auto cmd_buffer = device_.allocateCommandBuffers(cmd_alloc_info).front();
    if (begin) {
        auto begin_info =
            vk::CommandBufferBeginInfo().setFlags(usage).setPInheritanceInfo(p_inheritance_info);
        cmd_buffer.begin(begin_info);
    }

    return cmd_buffer;
}

void CommandPool::SubmitAndWait(vk::ArrayProxy<const vk::CommandBuffer> cmds, vk::Queue queue)
{
    for (auto& cmd : cmds)
        cmd.end();

    if (!queue)
        queue = default_queue_;
    CHECK(queue);
    auto submit_info =
        vk::SubmitInfo().setPCommandBuffers(cmds.data()).setCommandBufferCount(cmds.size());
    try {
        queue.submit(submit_info, /*fence =*/nullptr);
        queue.waitIdle();
    } catch (std::exception& e) {
        LOG(FATAL) << "command buffer submit failed" << e.what();
    }

    device_.freeCommandBuffers(cmd_pool_, cmds);
}

void DescriptorSetBindings::AddBinding(u32 binding, vk::DescriptorType type, u32 count,
                                       vk::ShaderStageFlags stage_flags,
                                       const vk::Sampler*   p_immutable_sampler)
{
    bindings_.emplace_back(Binding()
                               .setBinding(binding)
                               .setDescriptorType(type)
                               .setDescriptorCount(count)
                               .setStageFlags(stage_flags)
                               .setPImmutableSamplers(p_immutable_sampler));
}

vk::DescriptorType DescriptorSetBindings::GetType(u32 binding_index) const
{
    for (const auto& binding : bindings_) {
        if (binding.binding == binding_index)
            return binding.descriptorType;
    }
    return kInvalidType;
}

u32 DescriptorSetBindings::GetCount(u32 binding_index) const
{
    for (const auto& binding : bindings_) {
        if (binding.binding == binding_index)
            return binding.descriptorCount;
    }
    LOG_FATAL << "binding index not found";
    return -1;
}


vk::DescriptorSetLayout DescriptorSetBindings::MakeLayout(
    vk::Device device, vk::DescriptorSetLayoutCreateFlags flags) const
{
    auto ds_layout_ci =
        vk::DescriptorSetLayoutCreateInfo().setBindingCount(size()).setPBindings(data()).setFlags(
            flags);

    return device.createDescriptorSetLayout(ds_layout_ci);
}

vk::DescriptorPool DescriptorSetBindings::MakePool(vk::Device device, u32 max_sets) const
{
    std::vector<vk::DescriptorPoolSize> pool_sizes;
    for (const auto& binding : bindings_) {
        pool_sizes.emplace_back(vk::DescriptorPoolSize()
                                    .setType(binding.descriptorType)
                                    .setDescriptorCount(binding.descriptorCount * max_sets));
    }
    auto pool_ci = vk::DescriptorPoolCreateInfo()
                       .setMaxSets(max_sets)
                       .setPoolSizeCount(cast_u32(pool_sizes.size()))
                       .setPPoolSizes(pool_sizes.data());
    return device.createDescriptorPool(pool_ci);
}

Writer DescriptorSetBindings::MakeGeneralDSWrite(vk::DescriptorSet set, u32 binding_index,
                                                 u32 array_element) const
{
    auto ds_write = vk::WriteDescriptorSet()
                        .setDescriptorCount(1)
                        .setDescriptorType(GetType(binding_index))
                        .setDstBinding(binding_index)
                        .setDstSet(set)
                        .setDstArrayElement(array_element);
    LOG_IF(FATAL, ds_write.descriptorType == kInvalidType)
        << "binding index " << binding_index << " not found";
    return ds_write;
}

Writer DescriptorSetBindings::MakeWrite(vk::DescriptorSet set, u32 binding_index,
                                        const vk::DescriptorImageInfo* p_image_info,
                                        u32                            array_element) const
{
    auto ds_write = MakeGeneralDSWrite(set, binding_index, array_element);
    switch (ds_write.descriptorType) {
        case Type::eSampler:
        case Type::eCombinedImageSampler:
        case Type::eSampledImage:
        case Type::eStorageImage:
        case Type::eInputAttachment:
            ds_write.pImageInfo = p_image_info;
            break;
        default:
            LOG(FATAL) << "binding indexed " << binding_index << " isn't image";
    }
    return ds_write;
}

Writer DescriptorSetBindings::MakeWrite(vk::DescriptorSet set, u32 binding_index,
                                        const vk::DescriptorBufferInfo* p_buffer_info,
                                        u32                             array_element) const
{
    auto ds_write = MakeGeneralDSWrite(set, binding_index, array_element);
    switch (ds_write.descriptorType) {
        case Type::eStorageBuffer:
        case Type::eStorageBufferDynamic:
        case Type::eUniformBuffer:
        case Type::eUniformBufferDynamic:
            ds_write.pBufferInfo = p_buffer_info;
            break;
        default:
            LOG(FATAL) << "binding indexed " << binding_index << " isn't buffer";
    }
    return ds_write;
}

Writer DescriptorSetBindings::MakeWrite(vk::DescriptorSet set, u32 binding_index,
                                        const vk::BufferView* p_texel_buffer_view,
                                        u32                   array_element) const
{
    auto ds_write = MakeGeneralDSWrite(set, binding_index, array_element);
    switch (ds_write.descriptorType) {
        case Type::eUniformTexelBuffer:
        case Type::eStorageTexelBuffer:
            ds_write.pTexelBufferView = p_texel_buffer_view;
            break;
        default:
            LOG(FATAL) << "binding indexed " << binding_index << " isn't texel buffer";
    }
    return ds_write;
}

Writer DescriptorSetBindings::MakeWrite(
    vk::DescriptorSet set, u32 binding_index,
    const vk::WriteDescriptorSetAccelerationStructureKHR* p_accel, u32 array_element) const
{
    auto ds_write = MakeGeneralDSWrite(set, binding_index, array_element);
    switch (ds_write.descriptorType) {
        case Type::eAccelerationStructureKHR:
            ds_write.pNext = p_accel;
            break;
        default:
            LOG(FATAL) << "binding indexed " << binding_index << " isn't accel structure";
    }
    return ds_write;
}

Writer DescriptorSetBindings::MakeWrite(
    vk::DescriptorSet set, u32 binding_index,
    const vk::WriteDescriptorSetInlineUniformBlockEXT* p_inline_uniform, u32 array_element) const
{
    auto ds_write = MakeGeneralDSWrite(set, binding_index, array_element);
    switch (ds_write.descriptorType) {
        case Type::eInlineUniformBlockEXT:
            ds_write.pNext = p_inline_uniform;
            break;
        default:
            LOG(FATAL) << "binding indexed " << binding_index << " isn't inline uniform block";
    }
    return ds_write;
}

vk::WriteDescriptorSet DescriptorSetBindings::MakeGeneralDSWriteArray(vk::DescriptorSet set,
                                                                      u32 binding_index) const
{
    auto ds_write = vk::WriteDescriptorSet()
                        .setDescriptorCount(GetCount(binding_index))
                        .setDescriptorType(GetType(binding_index))
                        .setDstBinding(binding_index)
                        .setDstSet(set)
                        .setDstArrayElement(0);
    LOG_IF(FATAL, ds_write.descriptorType == kInvalidType)
        << "binding index " << binding_index << " not found";
    CHECK_NE(ds_write.descriptorCount, cast_u32(-1));
    return ds_write;
}

vk::WriteDescriptorSet DescriptorSetBindings::MakeWriteArray(
    vk::DescriptorSet set, u32 binding_index, const vk::DescriptorImageInfo* p_image_info) const
{
    auto ds_write = MakeGeneralDSWriteArray(set, binding_index);
    switch (ds_write.descriptorType) {
        case Type::eSampler:
        case Type::eCombinedImageSampler:
        case Type::eSampledImage:
        case Type::eStorageImage:
        case Type::eInputAttachment:
            ds_write.pImageInfo = p_image_info;
            break;
        default:
            LOG(FATAL) << "binding indexed " << binding_index << " isn't image";
    }
    return ds_write;
}

vk::WriteDescriptorSet DescriptorSetBindings::MakeWriteArray(
    vk::DescriptorSet set, u32 binding_index, const vk::DescriptorBufferInfo* p_buffer_info) const
{
    auto ds_write = MakeGeneralDSWriteArray(set, binding_index);
    switch (ds_write.descriptorType) {
        case Type::eStorageBuffer:
        case Type::eStorageBufferDynamic:
        case Type::eUniformBuffer:
        case Type::eUniformBufferDynamic:
            ds_write.pBufferInfo = p_buffer_info;
            break;
        default:
            LOG(FATAL) << "binding indexed " << binding_index << " isn't buffer";
    }
    return ds_write;
}

vk::WriteDescriptorSet DescriptorSetBindings::MakeWriteArray(
    vk::DescriptorSet set, u32 binding_index, const vk::BufferView* p_texel_buffer_view) const
{
    auto ds_write = MakeGeneralDSWriteArray(set, binding_index);
    switch (ds_write.descriptorType) {
        case Type::eUniformTexelBuffer:
        case Type::eStorageTexelBuffer:
            ds_write.pTexelBufferView = p_texel_buffer_view;
            break;
        default:
            LOG(FATAL) << "binding indexed " << binding_index << " isn't texel buffer";
    }
    return ds_write;
}

vk::WriteDescriptorSet DescriptorSetBindings::MakeWriteArray(
    vk::DescriptorSet set, u32 binding_index,
    const vk::WriteDescriptorSetAccelerationStructureKHR* p_accel) const
{
    auto ds_write = MakeGeneralDSWriteArray(set, binding_index);
    CHECK(ds_write.descriptorType == Type::eAccelerationStructureKHR);
    ds_write.pNext = p_accel;
    return ds_write;
}

vk::WriteDescriptorSet DescriptorSetBindings::MakeWriteArray(
    vk::DescriptorSet set, u32 binding_index,
    const vk::WriteDescriptorSetInlineUniformBlockEXT* p_inline_uniform) const
{
    auto ds_write = MakeGeneralDSWriteArray(set, binding_index);
    CHECK(ds_write.descriptorType == Type::eInlineUniformBlockEXT);
    ds_write.pNext = p_inline_uniform;
    return ds_write;
}

void BuggyRaytracingBuilder::Setup(const vk::Device& device, const UniqueMemoryAllocator* allocator,
                                   u32 queue_index)
{
    device_      = device;
    allocator_   = allocator;
    queue_index_ = queue_index;
}

void BuggyRaytracingBuilder::Destroy()
{
    for (auto& blas : blases_) {
        blas.accel_struct.DestroyFrom(device_);
    }
    tlas_.accel_struct.DestroyFrom(device_);
    instance_buffer_.DestroyFrom(device_);
    blases_.clear();
    tlas_ = {};
}

void BuggyRaytracingBuilder::BuildBlas(const std::vector<Blas>& blases, BuildASFlags flags)
{
    blases_ = blases;

    vk::DeviceSize max_scratch   = 0;
    bool           do_compaction = FlagsMatch(flags, BuildASFlagBits::eAllowCompaction);
    LOG(INFO) << (do_compaction ? "yes" : "no");

    std::vector<vk::DeviceSize> original_sizes;
    original_sizes.reserve(blases_.size());

    // Iterates over the groups of geometries, creating one BLAS for each group.
    for (auto& blas : blases_) {
        auto as_ci = vk::AccelerationStructureCreateInfoKHR()
                         .setType(vk::AccelerationStructureTypeKHR::eBottomLevel)
                         .setFlags(flags)
                         .setMaxGeometryCount(cast_u32(blas.as_create_geometry_info.size()))
                         .setPGeometryInfos(blas.as_create_geometry_info.data());

        blas.accel_struct = allocator_->MakeAccelStruct(as_ci);
        // TODO(zixun): debugger objectname.

        // Estimates the amount of scratch memory required to build the BLAS, and updates the size
        // of the scratch buffer that will be allocated to sequentially build all BLASes.
        auto mem_requirements_info =
            vk::AccelerationStructureMemoryRequirementsInfoKHR()
                .setType(vk::AccelerationStructureMemoryRequirementsTypeKHR::eBuildScratch)
                .setAccelerationStructure(blas.accel_struct.handle)
                .setBuildType(vk::AccelerationStructureBuildTypeKHR::eDevice);

        auto mem_requirements_2 =
            device_.getAccelerationStructureMemoryRequirementsKHR(mem_requirements_info);
        vk::DeviceSize scratch_size = mem_requirements_2.memoryRequirements.size;

        blas.flags  = flags;
        max_scratch = std::max(max_scratch, scratch_size);

        // Records original size.
        mem_requirements_info.type = vk::AccelerationStructureMemoryRequirementsTypeKHR::eObject;
        mem_requirements_2 =
            device_.getAccelerationStructureMemoryRequirementsKHR(mem_requirements_info);
        original_sizes.push_back(mem_requirements_2.memoryRequirements.size);

        LOG(INFO) << "scratch/original/max size = " << scratch_size << '/' << original_sizes.back()
                  << '/' << max_scratch;
    }

    UniqueMemoryBuffer scratch_buffer =
        allocator_->MakeBuffer(max_scratch, vk::BufferUsageFlagBits::eRayTracingKHR
                                                | vk::BufferUsageFlagBits::eShaderDeviceAddress);
    vk::BufferDeviceAddressInfo buffer_info{scratch_buffer.handle};
    vk::DeviceAddress           scratch_address = device_.getBufferAddress(buffer_info);

    // Queries size of compact BLAS.
    auto query_pool_ci = vk::QueryPoolCreateInfo()
                             .setQueryCount(blases_.size())
                             .setQueryType(vk::QueryType::eAccelerationStructureCompactedSizeKHR);
    auto query_pool = device_.createQueryPool(query_pool_ci);

    // Creates a command buffer containing all the BLAS builds.
    std::vector<VkCommandBuffer> all_cmd_buffers;
    all_cmd_buffers.reserve(blases_.size());

    nvvk::CommandPool cmdGen(device_, queue_index_);

    for (size_t i = 0; i < blases_.size(); ++i) {
        const auto blas = blases_[i];
        // auto       cmd_buffer = cmd_pool.MakeCmdBuffer();
        auto cmd_buffer = cmdGen.createCommandBuffer();
        all_cmd_buffers.push_back(cmd_buffer);

        const auto& p_geometry              = blas.as_geometry.data();
        auto        bottom_AS_geometry_info = vk::AccelerationStructureBuildGeometryInfoKHR()
                                           .setFlags(flags)
                                           .setUpdate(VK_FALSE)
                                           .setSrcAccelerationStructure(nullptr)
                                           .setDstAccelerationStructure(blas.accel_struct.handle)
                                           .setGeometryArrayOfPointers(VK_FALSE)
                                           .setGeometryCount(cast_u32(blas.as_geometry.size()))
                                           .setPpGeometries(&p_geometry);
        bottom_AS_geometry_info.scratchData.setDeviceAddress(scratch_address);

        std::vector<const ASBuildOffsetInfo*> p_build_offset;
        p_build_offset.reserve(blas.as_build_offset_info.size());
        for (size_t i = 0; i < blas.as_build_offset_info.size(); ++i)
            p_build_offset.push_back(&blas.as_build_offset_info[i]);

        // cmd_buffer.buildAccelerationStructureKHR(bottom_AS_geometry_info, p_build_offset);
        vkCmdBuildAccelerationStructureKHR(
            cmd_buffer, 1,
            reinterpret_cast<VkAccelerationStructureBuildGeometryInfoKHR*>(
                &bottom_AS_geometry_info),
            reinterpret_cast<const VkAccelerationStructureBuildOffsetInfoKHR**>(
                p_build_offset.data()));

        // Uses a barrier to ensure one build is finished before starting the next one.
        auto barrier = vk::MemoryBarrier()
                           .setSrcAccessMask(vk::AccessFlagBits::eAccelerationStructureWriteKHR)
                           .setDstAccessMask(vk::AccessFlagBits::eAccelerationStructureReadKHR);
        // cmd_buffer.pipelineBarrier(vk::PipelineStageFlagBits::eAccelerationStructureBuildKHR,
        // vk::PipelineStageFlagBits::eAccelerationStructureBuildKHR,
        ///*dependency_flags =*/{}, barrier, nullptr, nullptr);
        vkCmdPipelineBarrier(cmd_buffer, VK_PIPELINE_STAGE_ACCELERATION_STRUCTURE_BUILD_BIT_KHR,
                             VK_PIPELINE_STAGE_ACCELERATION_STRUCTURE_BUILD_BIT_KHR, 0, 1,
                             &(barrier.operator const VkMemoryBarrier&()), 0, nullptr, 0, nullptr);

        if (do_compaction)
            // cmd_buffer.writeAccelerationStructuresPropertiesKHR(
            //    blas.accel_struct.handle, vk::QueryType::eAccelerationStructureCompactedSizeKHR,
            //    query_pool, i);
            vkCmdWriteAccelerationStructuresPropertiesKHR(
                cmd_buffer, 1,
                reinterpret_cast<const VkAccelerationStructureKHR*>(&blas.accel_struct.handle),
                VK_QUERY_TYPE_ACCELERATION_STRUCTURE_COMPACTED_SIZE_KHR, query_pool, i);
    }
    cmdGen.submitAndWait(all_cmd_buffers);
    // cmd_pool.SubmitAndWait(all_cmd_buffers);
    all_cmd_buffers.clear();

    if (do_compaction) {
        CommandPool                 cmd_pool(device_, queue_index_);
        auto                        cmd_buffer = cmd_pool.MakeCmdBuffer();
        std::vector<vk::DeviceSize> compact_sizes(blases_.size());
        device_.getQueryPoolResults<vk::DeviceSize>(query_pool, 0, cast_u32(compact_sizes.size()),
                                                    compact_sizes, sizeof(vk::DeviceSize),
                                                    vk::QueryResultFlagBits::eWait);

        // Compacts accel strctures.
        std::vector<UniqueMemoryAccelStruct> cleanup_AS(blases_.size());
        u32                                  total_original_size = 0, total_compact_size = 0;
        for (int i = 0; i < blases_.size(); ++i) {
            total_original_size += cast_u32(original_sizes[i]);
            total_compact_size += cast_u32(compact_sizes[i]);

            // Creates a compact version of the accel structure.
            auto as_ci = vk::AccelerationStructureCreateInfoKHR()
                             .setCompactedSize(compact_sizes[i])
                             .setType(vk::AccelerationStructureTypeKHR::eBottomLevel)
                             .setFlags(flags);
            auto compact_as = allocator_->MakeAccelStruct(as_ci);

            // Copies the original BLAS to a compact version.
            auto copy_info = vk::CopyAccelerationStructureInfoKHR()
                                 .setSrc(blases_[i].accel_struct.handle)
                                 .setDst(compact_as.handle)
                                 .setMode(vk::CopyAccelerationStructureModeKHR::eCompact);
            cmd_buffer.copyAccelerationStructureKHR(copy_info);

            cleanup_AS[i]           = blases_[i].accel_struct;
            blases_[i].accel_struct = compact_as;
        }
        cmd_pool.SubmitAndWait(cmd_buffer);

        for (auto as : cleanup_AS) {
            device_.destroyAccelerationStructureKHR(as.handle);
            device_.freeMemory(as.memory);
        }

        LOG(INFO)
            << "Reducing from " << total_original_size << " to " << total_compact_size << ", "
            << total_original_size - total_compact_size << "("
            << (total_original_size - total_compact_size) * 100 / cast_f32(total_original_size)
            << ") less memory usage";
    }  // ends compaction

    device_.destroyQueryPool(query_pool);
    device_.destroyBuffer(scratch_buffer.handle);
    device_.freeMemory(scratch_buffer.memory);

    // allocator->finalizereleaseStaging
}

vk::AccelerationStructureInstanceKHR BuggyRaytracingBuilder::InstanceToVkGeometryInstanceKHR(
    const Instance& instance)
{
    Blas& blas = blases_[instance.blas_id];

    auto addr_info = vk::AccelerationStructureDeviceAddressInfoKHR().setAccelerationStructure(
        blas.accel_struct.handle);
    vk::DeviceAddress blas_addr = device_.getAccelerationStructureAddressKHR(addr_info);

    auto vi_instance = vk::AccelerationStructureInstanceKHR()
                           .setInstanceCustomIndex(instance.instance_id)
                           .setMask(instance.mask)
                           .setInstanceShaderBindingTableRecordOffset(instance.hit_group_id)
                           .setFlags(instance.flags)
                           .setAccelerationStructureReference(blas_addr);
    // In Vulkan, matrices are stored in row-major form, but glm uses column-major form.
    // The transform in vk::ASInstance contains only 12 float values, corresponding to the first 3
    // rows of the transform matrix, with the last row implicitly assumed to be [0, 0, 0, 1].
    glm::mat4 vk_transform = glm::transpose(instance.transform);
    static_assert(sizeof(vk_transform[0][0]) == sizeof(vi_instance.transform.matrix[0][0]), "");
    // Copies the first 3 rows of values.
    memcpy(&vi_instance.transform, &vk_transform, sizeof(vi_instance.transform));

    return vi_instance;
}

void BuggyRaytracingBuilder::BuildTlas(const std::vector<Instance>& instances, BuildASFlags flags)
{
    tlas_.flags = flags;

    auto geometry_create_type_info = ASCreateGeometryTypeInfo()
                                         .setGeometryType(vk::GeometryTypeKHR::eInstances)
                                         .setMaxPrimitiveCount(cast_u32(instances.size()))
                                         .setAllowsTransforms(VK_TRUE);

    auto as_ci = vk::AccelerationStructureCreateInfoKHR()
                     .setType(vk::AccelerationStructureTypeKHR::eTopLevel)
                     .setFlags(flags)
                     .setMaxGeometryCount(1)
                     .setPGeometryInfos(&geometry_create_type_info);

    // Creates the accel structure object and allocates the memory needed to host TLAS data.
    tlas_.accel_struct = allocator_->MakeAccelStruct(as_ci);

    auto memory_requirements_info =
        vk::AccelerationStructureMemoryRequirementsInfoKHR()
            .setType(vk::AccelerationStructureMemoryRequirementsTypeKHR::eBuildScratch)
            .setAccelerationStructure(tlas_.accel_struct.handle)
            .setBuildType(vk::AccelerationStructureBuildTypeKHR::eDevice);
    auto memory_requirements =
        device_.getAccelerationStructureMemoryRequirementsKHR(memory_requirements_info)
            .memoryRequirements;
    vk::DeviceSize scratch_size = memory_requirements.size;

    UniqueMemoryBuffer scratch_buffer =
        allocator_->MakeBuffer(scratch_size, vk::BufferUsageFlagBits::eRayTracingKHR
                                                 | vk::BufferUsageFlagBits::eShaderDeviceAddress);
    vk::DeviceAddress scratch_addr =
        device_.getBufferAddress(vk::BufferDeviceAddressInfo().setBuffer(scratch_buffer.handle));

    // For each instance, builds the corresponding instance descriptor.
    std::vector<vk::AccelerationStructureInstanceKHR> geometry_instances;
    geometry_instances.reserve(instances.size());
    for (const auto& instance : instances)
        geometry_instances.push_back(InstanceToVkGeometryInstanceKHR(instance));

    // Builds the TLAS.
    CommandPool       cmd_pool(device_, queue_index_);
    vk::CommandBuffer cmd_buffer = cmd_pool.MakeCmdBuffer();

    vk::DeviceSize instance_descs_size_bytes =
        instances.size() * sizeof(vk::AccelerationStructureInstanceKHR);

    instance_buffer_ = allocator_->MakeBuffer(cmd_buffer, geometry_instances,
                                              vk::BufferUsageFlagBits::eRayTracingKHR
                                                  | vk::BufferUsageFlagBits::eShaderDeviceAddress);
    vk::DeviceAddress instance_addr =
        device_.getBufferAddress(vk::BufferDeviceAddressInfo(instance_buffer_.handle));

    // Ensures the copy of the instance buffer are copied before executing the TLAS build.
    auto barrier = vk::MemoryBarrier()
                       .setSrcAccessMask(vk::AccessFlagBits::eTransferWrite)
                       .setDstAccessMask(vk::AccessFlagBits::eAccelerationStructureWriteKHR);
    cmd_buffer.pipelineBarrier(vk::PipelineStageFlagBits::eTransfer,
                               vk::PipelineStageFlagBits::eAccelerationStructureBuildKHR, {},
                               barrier, nullptr, nullptr);

    // Builds the TLAS.
    auto geometry_data                         = vk::AccelerationStructureGeometryDataKHR();
    geometry_data.instances.arrayOfPointers    = VK_FALSE;
    geometry_data.instances.data.deviceAddress = instance_addr;
    auto tlas_geometry =
        ASGeometry().setGeometryType(vk::GeometryTypeKHR::eInstances).setGeometry(geometry_data);

    const vk::AccelerationStructureGeometryKHR* p_geometry = &tlas_geometry;
    auto tlas_info = vk::AccelerationStructureBuildGeometryInfoKHR()
                         .setFlags(flags)
                         .setUpdate(VK_FALSE)
                         .setSrcAccelerationStructure(nullptr)
                         .setDstAccelerationStructure(tlas_.accel_struct.handle)
                         .setGeometryArrayOfPointers(VK_FALSE)
                         .setGeometryCount(1)
                         .setPpGeometries(&p_geometry);
    tlas_info.scratchData.setDeviceAddress(scratch_addr);

    auto build_offset_info =
        vk::AccelerationStructureBuildOffsetInfoKHR(cast_u32(instances.size()), 0, 0, 0);
    const vk::AccelerationStructureBuildOffsetInfoKHR* p_build_offset_info = &build_offset_info;

    cmd_buffer.buildAccelerationStructureKHR(tlas_info, p_build_offset_info);

    cmd_pool.SubmitAndWait(cmd_buffer);
    device_.destroyBuffer(scratch_buffer.handle);
    device_.freeMemory(scratch_buffer.memory);
}

void BuggyRaytracingBuilder::UpdateTlasMatrices(const std::vector<Instance>& instances)
{
    using vkBU = vk::BufferUsageFlagBits;
    using vkMP = vk::MemoryPropertyFlagBits;

    vk::DeviceSize buffer_size = instances.size() * sizeof(vk::AccelerationStructureInstanceKHR);

    // Creates a staging buffer on the host to upload the new instance data.
    UniqueMemoryBuffer staging_buffer =
        allocator_->MakeBuffer(buffer_size, vkBU::eTransferSrc,
                               vkMP::eHostVisible | vkMP::eHostCoherent);

    // Copies the instance data to the staging buffer.
    auto* vk_inst_memory = reinterpret_cast<vk::AccelerationStructureInstanceKHR*>(
        device_.mapMemory(staging_buffer.memory, 0, VK_WHOLE_SIZE));

    for (int i = 0; i < instances.size(); ++i) {
        vk_inst_memory[i] = InstanceToVkGeometryInstanceKHR(instances[i]);
    }
    device_.unmapMemory(staging_buffer.memory);

    // Computes the amount of scratch memory required by the AS build to update.
    auto mem_requirements_info =
        vk::AccelerationStructureMemoryRequirementsInfoKHR()
            .setType(vk::AccelerationStructureMemoryRequirementsTypeKHR::eUpdateScratch)
            .setAccelerationStructure(tlas_.accel_struct.handle)
            .setBuildType(vk::AccelerationStructureBuildTypeKHR::eDevice);
    auto mem_requirements =
        device_.getAccelerationStructureMemoryRequirementsKHR(mem_requirements_info)
            .memoryRequirements;
    vk::DeviceSize scratch_size = mem_requirements.size;

    // Allocates the scratch buffer.
    UniqueMemoryBuffer scratch_buffer =
        allocator_->MakeBuffer(scratch_size, vkBU::eRayTracingKHR | vkBU::eShaderDeviceAddress);
    vk::DeviceAddress scratch_address =
        device_.getBufferAddress(vk::BufferDeviceAddressInfo(scratch_buffer.handle));

    // Updates the instance buffer on the device side and builds the TLAS.
    CommandPool cmd_pool(device_, queue_index_);
    auto        cmd_buffer = cmd_pool.MakeCmdBuffer();

    // Command #1: copies data from the staging buffer to the instances buffer.
    auto region = vk::BufferCopy().setSrcOffset(0).setDstOffset(0).setSize(buffer_size);
    cmd_buffer.copyBuffer(staging_buffer.handle, instance_buffer_.handle, region);

    vk::DeviceAddress instance_addr =
        device_.getBufferAddress(vk::BufferDeviceAddressInfo(instance_buffer_.handle));

    // Command #2: Ensures that instance buffer copy is complete before initiating the TLAS build.
    auto memory_barrier = vk::MemoryBarrier()
                              .setSrcAccessMask(vk::AccessFlagBits::eTransferWrite)
                              .setDstAccessMask(vk::AccessFlagBits::eAccelerationStructureWriteKHR);
    cmd_buffer.pipelineBarrier(vk::PipelineStageFlagBits::eTransfer,
                               vk::PipelineStageFlagBits::eAccelerationStructureBuildKHR,
                               /*dependencies=*/{}, memory_barrier, nullptr, nullptr);

    vk::AccelerationStructureGeometryDataKHR geometry_data;
    geometry_data.instances.arrayOfPointers    = VK_FALSE;
    geometry_data.instances.data.deviceAddress = instance_addr;
    auto tlas_geometry                         = vk::AccelerationStructureGeometryKHR()
                             .setGeometryType(vk::GeometryTypeKHR::eInstances)
                             .setGeometry(geometry_data);
    const auto* p_tlas_geometry = &tlas_geometry;

    auto tlas_geometry_info = vk::AccelerationStructureBuildGeometryInfoKHR()
                                  .setFlags(tlas_.flags)
                                  .setUpdate(VK_TRUE)
                                  .setSrcAccelerationStructure(tlas_.accel_struct.handle)
                                  .setDstAccelerationStructure(tlas_.accel_struct.handle)
                                  .setGeometryArrayOfPointers(VK_FALSE)
                                  .setGeometryCount(1)
                                  .setPpGeometries(&p_tlas_geometry);
    tlas_geometry_info.scratchData.setDeviceAddress(scratch_address);

    u32  num_instances = cast_u32(instances.size());
    auto build_offset_info =
        vk::AccelerationStructureBuildOffsetInfoKHR().setPrimitiveCount(num_instances);
    const auto* p_build_offset_info = &build_offset_info;

    // Command #3: Initiates accel structure building/updating.
    cmd_buffer.buildAccelerationStructureKHR(tlas_geometry_info, &build_offset_info);
    cmd_pool.SubmitAndWait(cmd_buffer);

    device_.destroyBuffer(scratch_buffer.handle);
    device_.destroyBuffer(staging_buffer.handle);
    device_.freeMemory(scratch_buffer.memory);
    device_.freeMemory(staging_buffer.memory);
}

void BuggyRaytracingBuilder::UpdateBlas(u32 blas_index)
{
    Blas& blas = blases_[blas_index];
    auto  mem_requirements =
        device_
            .getAccelerationStructureMemoryRequirementsKHR(
                vk::AccelerationStructureMemoryRequirementsInfoKHR()
                    .setType(vk::AccelerationStructureMemoryRequirementsTypeKHR::eUpdateScratch)
                    .setAccelerationStructure(blas.accel_struct.handle)
                    .setBuildType(vk::AccelerationStructureBuildTypeKHR::eDevice))
            .memoryRequirements;
    vk::DeviceSize scratch_size = mem_requirements.size;

    UniqueMemoryBuffer scratch_buffer =
        allocator_->MakeBuffer(scratch_size, vk::BufferUsageFlagBits::eRayTracingKHR
                                                 | vk::BufferUsageFlagBits::eShaderDeviceAddress);
    vk::DeviceAddress scratch_addr =
        device_.getBufferAddress(vk::BufferDeviceAddressInfo().setBuffer(scratch_buffer.handle));

    const auto* p_geometry             = blas.as_geometry.data();
    auto        as_build_geometry_info = vk::AccelerationStructureBuildGeometryInfoKHR()
                                      .setType(vk::AccelerationStructureTypeKHR::eBottomLevel)
                                      .setFlags(blas.flags)
                                      .setUpdate(VK_TRUE)
                                      .setSrcAccelerationStructure(blas.accel_struct.handle)
                                      .setDstAccelerationStructure(blas.accel_struct.handle)
                                      .setGeometryArrayOfPointers(VK_FALSE)
                                      .setGeometryCount(cast_u32(blas.as_geometry.size()))
                                      .setPpGeometries(&p_geometry);
    as_build_geometry_info.scratchData.deviceAddress = scratch_addr;

    std::vector<const vk::AccelerationStructureBuildOffsetInfoKHR*> p_build_offset(
        blas.as_build_offset_info.size());
    for (size_t i = 0; i < blas.as_build_offset_info.size(); ++i)
        p_build_offset[i] = &blas.as_build_offset_info[i];

    // Updates the instance buffer on the device and builds the BLAS.
    CommandPool       cmd_pool(device_, queue_index_);
    vk::CommandBuffer cmd_buffer = cmd_pool.MakeCmdBuffer();

    cmd_buffer.buildAccelerationStructureKHR(as_build_geometry_info, p_build_offset);
    cmd_pool.SubmitAndWait(cmd_buffer);

    device_.destroyBuffer(scratch_buffer.handle);
    device_.freeMemory(scratch_buffer.memory);
}


void CmdGenerateMipmaps(vk::CommandBuffer cmd_buffer, const vk::Image& image,
                        vk::Format image_format, const vk::Extent2D& size, uint32_t mipLevels)
{
    using vkPS   = vk::PipelineStageFlagBits;
    auto barrier = vk::ImageMemoryBarrier()
                       .setImage(image)
                       .setOldLayout(vk::ImageLayout::eShaderReadOnlyOptimal)
                       .setNewLayout(vk::ImageLayout::eTransferSrcOptimal)
                       .setDstAccessMask(vk::AccessFlagBits::eTransferRead);
    barrier.subresourceRange.setBaseArrayLayer(0)
        .setAspectMask(vk::ImageAspectFlagBits::eColor)
        .setBaseMipLevel(0)
        .setLayerCount(1)
        .setLevelCount(1);

    cmd_buffer.pipelineBarrier(vkPS::eTransfer, vkPS::eTransfer, {}, nullptr, nullptr, barrier);
    i32 mip_width  = size.width;
    i32 mip_height = size.height;

    for (u32 i = 1; i < mipLevels; ++i) {
        auto blit          = vk::ImageBlit();
        blit.srcOffsets[0] = vk::Offset3D{0, 0, 0};
        blit.srcOffsets[1] = vk::Offset3D{mip_width, mip_height, 1};
        blit.dstOffsets[0] = vk::Offset3D{0, 0, 0};
        blit.dstOffsets[1] =
            vk::Offset3D{mip_width > 1 ? mip_width / 2 : 1, mip_height > 1 ? mip_height / 2 : 1, 1};
        blit.srcSubresource = vk::ImageSubresourceLayers(vk::ImageAspectFlagBits::eColor)
                                  .setMipLevel(i - 1)
                                  .setBaseArrayLayer(0)
                                  .setLayerCount(1);
        blit.dstSubresource = blit.srcSubresource;
        blit.dstSubresource.setMipLevel(i);

        cmd_buffer.blitImage(image, vk::ImageLayout::eTransferSrcOptimal, image,
                             vk::ImageLayout::eTransferDstOptimal, blit, vk::Filter::eLinear);

        // Next level..
        if (i + 1 < mipLevels) {
            barrier.subresourceRange.baseMipLevel = i;
            barrier.setOldLayout(vk::ImageLayout::eTransferDstOptimal)
                .setNewLayout(vk::ImageLayout::eTransferSrcOptimal)
                .setSrcAccessMask(vk::AccessFlagBits::eTransferWrite)
                .setDstAccessMask(vk::AccessFlagBits::eTransferRead);
            cmd_buffer.pipelineBarrier(vkPS::eTransfer, vkPS::eTransfer, {}, nullptr, nullptr,
                                       barrier);
        }

        if (mip_width > 1)
            mip_width /= 2;
        if (mip_height > 1)
            mip_height /= 2;
    }

    // Transitions all mip levels into a shader read-only optimal layout.
    barrier.subresourceRange.setBaseMipLevel(0).setLevelCount(mipLevels);
    barrier.setOldLayout(vk::ImageLayout::eUndefined)
        .setNewLayout(vk::ImageLayout::eShaderReadOnlyOptimal)
        .setSrcAccessMask({})
        .setDstAccessMask(vk::AccessFlagBits::eShaderRead);
    cmd_buffer.pipelineBarrier(vkPS::eTransfer, vkPS::eFragmentShader, {}, nullptr, nullptr,
                               barrier);

    return;
}

void CmdBarrierImageLayout(vk::CommandBuffer cmd_buffer, vk::Image image,
                           vk::ImageLayout old_layout, vk::ImageLayout new_layout,
                           const vk::ImageSubresourceRange& sub_range)
{
    auto image_memory_barrier = vk::ImageMemoryBarrier()
                                    .setOldLayout(old_layout)
                                    .setNewLayout(new_layout)
                                    .setImage(image)
                                    .setSubresourceRange(sub_range)
                                    .setSrcAccessMask(AccessFlagsForImageLayout(old_layout))
                                    .setDstAccessMask(AccessFlagsForImageLayout(new_layout));
    vk::PipelineStageFlags src_stage = PipelineStageForLayout(old_layout);
    vk::PipelineStageFlags dst_stage = PipelineStageForLayout(new_layout);

    cmd_buffer.pipelineBarrier(src_stage, dst_stage, {}, nullptr, nullptr, image_memory_barrier);
}

void CmdBarrierImageLayout(vk::CommandBuffer cmd_buffer, vk::Image image,
                           vk::ImageLayout old_layout, vk::ImageLayout new_layout,
                           vk::ImageAspectFlags aspect_mask)
{
    auto subresource_range = vk::ImageSubresourceRange()
                                 .setAspectMask(aspect_mask)
                                 .setLevelCount(1)
                                 .setLayerCount(1)
                                 .setBaseMipLevel(0)
                                 .setBaseArrayLayer(0);
    CmdBarrierImageLayout(cmd_buffer, image, old_layout, new_layout, subresource_range);
}

vk::ImageCreateInfo MakeImage2DCreateInfo(const vk::Extent2D& size, vk::Format format,
                                          vk::ImageUsageFlags usage)
{
    return vk::ImageCreateInfo()
        .setImageType(vk::ImageType::e2D)
        .setFormat(format)
        .setSamples(vk::SampleCountFlagBits::e1)
        .setMipLevels(1)
        .setArrayLayers(1)
        .setExtent(vk::Extent3D(size, 1))
        .setUsage(usage | vk::ImageUsageFlagBits::eTransferSrc
                  | vk::ImageUsageFlagBits::eTransferDst);
}

vk::ImageViewCreateInfo MakeImage2DViewCreateInfo(const vk::Image& image, vk::Format format,
                                                  vk::ImageAspectFlags aspect_flags, u32 levels,
                                                  const void* p_next_image_view)
{
    return vk::ImageViewCreateInfo()
        .setPNext(p_next_image_view)
        .setImage(image)
        .setViewType(vk::ImageViewType::e2D)
        .setFormat(format)
        .setSubresourceRange(vk::ImageSubresourceRange(aspect_flags, 0, levels, 0, 1));
}

vk::ImageViewCreateInfo MakeImageViewCreateInfo(const vk::Image&           image,
                                                const vk::ImageCreateInfo& image_ci)
{
    vk::ImageViewType view_type = {};
    switch (image_ci.imageType) {
        case vk::ImageType::e1D:
        case vk::ImageType::e2D:
        case vk::ImageType::e3D:
            // The underlying values are exactly the same under these 3 cases.
            view_type = static_cast<vk::ImageViewType>(VkFlags(image_ci.imageType));
            break;
        default:
            LOG(FATAL);
    }

    auto subresource_range =
        vk::ImageSubresourceRange(vk::ImageAspectFlagBits::eColor, 0, VK_REMAINING_MIP_LEVELS, 0,
                                  VK_REMAINING_ARRAY_LAYERS);

    return vk::ImageViewCreateInfo()
        .setImage(image)
        .setFormat(image_ci.format)
        .setViewType(view_type)
        .setSubresourceRange(subresource_range);
}

vk::RenderPass MakeRenderPass(vk::Device&                    device,
                              const std::vector<vk::Format>& color_attachment_formats,
                              vk::Format depth_attachment_format, u32 subpass_count,
                              bool clear_color, bool clear_depth, vk::ImageLayout initial_layout,
                              vk::ImageLayout final_layout)
{
    std::vector<vk::AttachmentDescription> all_attachments;
    std::vector<vk::AttachmentReference>   color_attachment_refs;
    bool has_depth = (depth_attachment_format != vk::Format::eUndefined);

    for (auto format : color_attachment_formats) {
        auto color_attachment = vk::AttachmentDescription()
                                    .setFormat(format)
                                    .setSamples(vk::SampleCountFlagBits::e1)
                                    .setLoadOp(clear_color ? vk::AttachmentLoadOp::eClear :
                                                             vk::AttachmentLoadOp::eDontCare)
                                    .setStoreOp(vk::AttachmentStoreOp::eStore)
                                    .setStencilLoadOp(vk::AttachmentLoadOp::eDontCare)
                                    .setStencilStoreOp(vk::AttachmentStoreOp::eDontCare)
                                    .setInitialLayout(initial_layout)
                                    .setFinalLayout(final_layout);
        auto color_attachment_ref = vk::AttachmentReference()
                                        .setAttachment(cast_u32(all_attachments.size()))
                                        .setLayout(vk::ImageLayout::eColorAttachmentOptimal);

        all_attachments.push_back(color_attachment);
        color_attachment_refs.push_back(color_attachment_ref);
    }
    auto depth_attachment_ref = vk::AttachmentReference();
    if (has_depth) {
        auto depth_attachment = vk::AttachmentDescription()
                                    .setFormat(depth_attachment_format)
                                    .setSamples(vk::SampleCountFlagBits::e1)
                                    .setLoadOp(clear_depth ? vk::AttachmentLoadOp::eClear :
                                                             vk::AttachmentLoadOp::eDontCare)
                                    .setStoreOp(vk::AttachmentStoreOp::eStore);
        depth_attachment.setStencilLoadOp(vk::AttachmentLoadOp::eDontCare)
            .setStencilStoreOp(vk::AttachmentStoreOp::eDontCare)
            .setInitialLayout(vk::ImageLayout::eDepthStencilAttachmentOptimal)
            .setFinalLayout(vk::ImageLayout::eDepthStencilAttachmentOptimal);

        depth_attachment_ref.setAttachment(cast_u32(all_attachments.size()))
            .setLayout(vk::ImageLayout::eDepthStencilAttachmentOptimal);

        all_attachments.push_back(depth_attachment);
    }

    std::vector<vk::SubpassDescription> subpasses;
    std::vector<vk::SubpassDependency> subpass_dependencies;

    for (u32 i = 0; i < subpass_count; ++i) {
        auto subpass = vk::SubpassDescription({}, vk::PipelineBindPoint::eGraphics)
                           .setColorAttachmentCount(cast_u32(color_attachment_refs.size()))
                           .setPColorAttachments(color_attachment_refs.data())
                           .setPDepthStencilAttachment(has_depth ? &depth_attachment_ref : nullptr);
        auto dependency = vk::SubpassDependency()
                              .setSrcSubpass(i == 0 ? VK_SUBPASS_EXTERNAL : i - 1)
                              .setDstSubpass(i)
                              .setSrcStageMask(vk::PipelineStageFlagBits::eColorAttachmentOutput)
                              .setDstStageMask(vk::PipelineStageFlagBits::eColorAttachmentOutput)
                              .setSrcAccessMask({})
                              .setDstAccessMask(vk::AccessFlagBits::eColorAttachmentWrite);

        subpasses.push_back(subpass);
        subpass_dependencies.push_back(dependency);
    }

    auto render_pass_ci = vk::RenderPassCreateInfo();
    render_pass_ci.setAttachmentCount(cast_u32(all_attachments.size()))
        .setPAttachments(all_attachments.data());
    render_pass_ci.setSubpassCount(cast_u32(subpasses.size())).setPSubpasses(subpasses.data());
    render_pass_ci.setDependencyCount(cast_u32(subpass_dependencies.size()))
        .setPDependencies(subpass_dependencies.data());
    return device.createRenderPass(render_pass_ci);
}

}  // namespace vkpbr
