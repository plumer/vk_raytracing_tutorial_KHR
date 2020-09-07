#include "vk_memory.h"

#include "logging.h"
#include "vk_utils.h"

namespace {



}  // namespace

namespace vkpbr {

void UniqueMemoryAllocator::Setup(const vk::Device& device, const vk::PhysicalDevice& gpu)
{
    device_ = device;
    gpu_    = gpu;
}


UniqueMemoryImage UniqueMemoryAllocator::MakeImage(const vk::ImageCreateInfo& image_ci,
                                                   vk::MemoryPropertyFlags    memory_usage)
{
    UniqueMemoryImage result;

    result.handle = device_.createImage(image_ci);

    vk::MemoryRequirements2          memory_requirements;
    vk::MemoryDedicatedRequirements  dedicated_requirements;
    vk::ImageMemoryRequirementsInfo2 image_requirements_info;

    image_requirements_info.image = result.handle;
    memory_requirements.pNext     = &dedicated_requirements;
    device_.getImageMemoryRequirements2(&image_requirements_info, &memory_requirements);

    // Allocates memory.
    auto memory_alloc_info =
        vk::MemoryAllocateInfo()
            .setAllocationSize(memory_requirements.memoryRequirements.size)
            .setMemoryTypeIndex(GetMemoryTypeIndex(
                memory_requirements.memoryRequirements.memoryTypeBits, memory_usage));
    result.memory = device_.allocateMemory(memory_alloc_info);
    CHECK(result.memory);

    // Binds memory to image.
    device_.bindImageMemory(result.handle, result.memory, /*offset = */ 0);

    return result;
}

UniqueMemoryImage UniqueMemoryAllocator::MakeImage(const vk::CommandBuffer& cmd_buffer, size_t size,
                                                   const void*                data,
                                                   const vk::ImageCreateInfo& image_ci,
                                                   const vk::ImageLayout      layout)
{
    UniqueMemoryImage result = MakeImage(image_ci, vk::MemoryPropertyFlagBits::eDeviceLocal);

    if (data != nullptr) {
        UniqueMemoryBuffer staging_buffer = MakeBuffer(
            size, vk::BufferUsageFlagBits::eTransferSrc,
            vk::MemoryPropertyFlagBits::eHostCoherent | vk::MemoryPropertyFlagBits::eHostVisible);
        staging_buffers_.push_back(staging_buffer);

        // Copies data to the staging buffer.
        void* mapped = device_.mapMemory(staging_buffer.memory, /*offset = */ 0, size);
        memcpy(mapped, data, size);
        device_.unmapMemory(staging_buffer.memory);

        // Copies buffer to the image.
        auto subresource_range = vk::ImageSubresourceRange()
                                     .setAspectMask(vk::ImageAspectFlagBits::eColor)
                                     .setBaseArrayLayer(0)
                                     .setBaseMipLevel(0)
                                     .setLayerCount(1)
                                     .setLevelCount(image_ci.mipLevels);
        // TODO barrier image layout
        CmdBarrierImageLayout(cmd_buffer, result.handle, vk::ImageLayout::eUndefined,
                              vk::ImageLayout::eTransferDstOptimal, subresource_range);

        auto buffer_copy_region = vk::BufferImageCopy().setImageExtent(image_ci.extent);
        buffer_copy_region.imageSubresource.setAspectMask(vk::ImageAspectFlagBits::eColor)
            .setLayerCount(1);
        cmd_buffer.copyBufferToImage(staging_buffer.handle, result.handle,
                                     vk::ImageLayout::eTransferDstOptimal, buffer_copy_region);

        // Sets final image layout.
        subresource_range.setLevelCount(1);
        CmdBarrierImageLayout(cmd_buffer, result.handle, vk::ImageLayout::eTransferDstOptimal,
                              layout, subresource_range);
    } else {
        CmdBarrierImageLayout(cmd_buffer, result.handle, vk::ImageLayout::eUndefined, layout,
                              vk::ImageAspectFlagBits::eColor);
    }
    return result;
}

UniqueMemoryTexture UniqueMemoryAllocator::MakeTexture(const UniqueMemoryImage&       image,
                                                       const vk::ImageViewCreateInfo& image_view_ci,
                                                       const vk::SamplerCreateInfo&   sampler_ci)
{
    UniqueMemoryTexture result;
    result.handle                 = image.handle;
    result.memory                 = image.memory;
    result.descriptor.imageLayout = vk::ImageLayout::eShaderReadOnlyOptimal;

    CHECK_EQ(image_view_ci.image, result.handle);
    result.descriptor.imageView = device_.createImageView(image_view_ci);

    //if (sampler_ci.flags != vk::SamplerCreateFlags()) {}
    result.descriptor.sampler = device_.createSampler(sampler_ci);
    return result;
}

u32 UniqueMemoryAllocator::GetMemoryTypeIndex(u32                     type_bits,
                                              vk::MemoryPropertyFlags desired_properties) const
{
    // GPU memory properties should have been a member of the class, but for some weird issues of
    // MSVC this isn't easy to do. Therefore, as a workaround, gpu handle is cached statically.
    thread_local static auto cached_gpu_handle = gpu_;
    thread_local static auto memory_properties = cached_gpu_handle.getMemoryProperties();
    // Checks for gpu handle change every time this method is invoked; and if cache is outdated,
    // updates the GPU handle and re-retrieves the memory properties.
    if (gpu_ != cached_gpu_handle) {
        cached_gpu_handle = gpu_;
        memory_properties = cached_gpu_handle.getMemoryProperties();
    }

    for (u32 i = 0; i < memory_properties.memoryTypeCount; ++i) {
        bool i_th_bit_matched = (type_bits & (i << i)) > 0;
        bool i_th_property_satisfied =
            (memory_properties.memoryTypes[i].propertyFlags & desired_properties)
            == desired_properties;
        if (i_th_bit_matched && i_th_property_satisfied) {
            return i;
        }
    }
    LOG(FATAL) << "Unable to find memory type " << vk::to_string(desired_properties);
    return ~0u;
}

void UniqueMemoryAllocator::ReleaseAllStagingBuffers()
{
    for (auto& buffer : staging_buffers_) {
        buffer.DestroyFrom(device_);
    }
}

UniqueMemoryAccelStruct UniqueMemoryAllocator::MakeAccelStruct(
    const vk::AccelerationStructureCreateInfoKHR& accel_struct_ci) const
{
    UniqueMemoryAccelStruct result_accel;
    // 1. Creates the accel structure handle.
    result_accel.handle = device_.createAccelerationStructureKHR(accel_struct_ci);


    // 2. Find memory requirements.
    vk::AccelerationStructureMemoryRequirementsInfoKHR mem_requirements_info;
    mem_requirements_info.setAccelerationStructure(result_accel.handle)
        .setBuildType(vk::AccelerationStructureBuildTypeKHR::eDevice)
        .setType(vk::AccelerationStructureMemoryRequirementsTypeKHR::eObject);
    auto memory_requirements =
        device_.getAccelerationStructureMemoryRequirementsKHR(mem_requirements_info)
            .memoryRequirements;

    // seems useless
    // auto memory_allocate_flags_info =
    // vk::MemoryAllocateFlagsInfo().setFlags(vk::MemoryAllocateFlagBits::eDeviceAddress);

    // 3. Allocates memory.
    auto memory_alloc_info =
        vk::MemoryAllocateInfo()
            .setAllocationSize(memory_requirements.size)
            .setMemoryTypeIndex(GetMemoryTypeIndex(memory_requirements.memoryTypeBits,
                                                   vk::MemoryPropertyFlagBits::eDeviceLocal));
    result_accel.memory = device_.allocateMemory(memory_alloc_info);

    device_.bindAccelerationStructureMemoryKHR(vk::BindAccelerationStructureMemoryInfoKHR()
                                                   .setAccelerationStructure(result_accel.handle)
                                                   .setMemory(result_accel.memory)
                                                   .setMemoryOffset(0));
    return result_accel;
}

UniqueMemoryBuffer UniqueMemoryAllocator::MakeBuffer(vk::DeviceSize          size,
                                                     vk::BufferUsageFlags    usage,
                                                     vk::MemoryPropertyFlags memory_usage) const
{
    CHECK_GT(size, 0);
    auto               buffer_ci = vk::BufferCreateInfo().setSize(size).setUsage(usage);
    UniqueMemoryBuffer result_buffer;
    result_buffer.handle = device_.createBuffer(buffer_ci);

    // Finds memory requirements.
    vk::MemoryRequirements2           mem_requirements;
    vk::MemoryDedicatedRequirements   dedicated_requirements;
    vk::BufferMemoryRequirementsInfo2 buffer_requirements;

    buffer_requirements.buffer = result_buffer.handle;
    mem_requirements.pNext     = &dedicated_requirements;
    mem_requirements           = device_.getBufferMemoryRequirements2(buffer_requirements);

    vk::MemoryAllocateFlagsInfo mem_flags_info;
    if (usage & vk::BufferUsageFlagBits::eShaderDeviceAddress)
        mem_flags_info.flags = vk::MemoryAllocateFlagBits::eDeviceAddress;

    // Allocates memory.
    auto mem_alloc_info = vk::MemoryAllocateInfo()
                              .setAllocationSize(mem_requirements.memoryRequirements.size)
                              .setMemoryTypeIndex(GetMemoryTypeIndex(
                                  mem_requirements.memoryRequirements.memoryTypeBits, memory_usage))
                              .setPNext(&mem_flags_info);
    result_buffer.memory = device_.allocateMemory(mem_alloc_info);

    // Binds memory to buffer.
    device_.bindBufferMemory(result_buffer.handle, result_buffer.memory, /*memory_offset=*/0);

    return result_buffer;
}

void UniqueMemoryBuffer::DestroyFrom(const vk::Device& device)
{
    if (handle) {
        CHECK(memory) << "a non-null buffer should be paired with a non-null memory";
        device.destroyBuffer(handle);
        device.freeMemory(memory);
        handle = nullptr;
        memory = nullptr;
    } else {
        CHECK(!memory) << "a null buffer should be paired with a null memory";
    }
}

void UniqueMemoryAccelStruct::DestroyFrom(const vk::Device& device)
{
    if (handle) {
        CHECK(memory) << "a non-null accel struct should be paired with a non-null memory";
        device.destroyAccelerationStructureKHR(handle);
        device.freeMemory(memory);
        handle = nullptr;
        memory = nullptr;
    } else {
        CHECK(!memory) << "a null accel struct should be paired with a null memory";
    }
}

void UniqueMemoryImage::DestroyFrom(const vk::Device& device)
{
    device.destroyImage(handle);
    device.freeMemory(memory);
}

void UniqueMemoryTexture::DestroyFrom(const vk::Device& device)
{
    device.destroyImage(handle);
    device.freeMemory(memory);
    if (descriptor.sampler)
        device.destroySampler(descriptor.sampler);
    if (descriptor.imageView) 
        device.destroyImageView(descriptor.imageView);
}

}  // namespace vkpbr
