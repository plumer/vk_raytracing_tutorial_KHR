#include "vk_memory.h"

#include "logging.h"

namespace vkpbr {

void UniqueMemoryAllocator::Setup(const vk::Device& device, const vk::PhysicalDevice& gpu)
{
    device_ = device;
    gpu_    = gpu;
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

void UniqueMemoryAllocator::ReleaseAllStagingBuffers() {
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

}  // namespace vkpbr
