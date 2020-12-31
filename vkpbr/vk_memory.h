#ifndef NVCOPY_VK_MEMORY_H_
#define NVCOPY_VK_MEMORY_H_

#ifndef VULKAN_HPP
#include <vulkan/vulkan.hpp>
#endif

#include "types.h"

namespace vkpbr {

struct UniqueMemoryBuffer {
    vk::Buffer       handle;
    vk::DeviceMemory memory;
    /**
     * Destroys the buffer handle and frees the memory.
     *
     * \param device The device from which the buffer is created and the memory allocated.
     */
    void DestroyFrom(const vk::Device& device);
};

struct UniqueMemoryAccelStruct {
    vk::AccelerationStructureKHR handle;
    UniqueMemoryBuffer           buffer;
    void                         DestroyFrom(const vk::Device& device);
};

struct UniqueMemoryImage {
    vk::Image        handle;
    vk::DeviceMemory memory;
    void             DestroyFrom(const vk::Device& device);
};

struct UniqueMemoryTexture {
    vk::Image handle;
    vk::DeviceMemory memory;
    vk::DescriptorImageInfo descriptor;

    void DestroyFrom(const vk::Device& device);
};

// Computes number of bytes occupied by the data in the container.
template <typename T>
inline size_t DataSize(const std::vector<T>& std_vector)
{
    return std_vector.size() * sizeof(T);
}

/**
 * A utility class for allocating Vulkan resources that owns memory exclusively.
 * That means, every buffer/image/texture/accelstruct allocated by this class is bound to a unique
 * vk::DeviceMemory, not shared with other objects.
 */
class UniqueMemoryAllocator
{
  public:
    UniqueMemoryAllocator()                             = default;
    UniqueMemoryAllocator(const UniqueMemoryAllocator&) = delete;
    UniqueMemoryAllocator& operator=(const UniqueMemoryAllocator&) = delete;
    ~UniqueMemoryAllocator() { ReleaseAllStagingBuffers(); }

    void Setup(const vk::Device& device, const vk::PhysicalDevice& gpu);
    bool ReadyToUse() const { return device_ && gpu_; }
    void ReleaseAllStagingBuffers();

    UniqueMemoryAccelStruct MakeAccelStruct(
        const vk::AccelerationStructureCreateInfoKHR& accel_struct_ci) const;

    // Buffers
    // --------------------------------------------------------------------------------------------

    UniqueMemoryBuffer MakeBuffer(
        vk::DeviceSize size, vk::BufferUsageFlags usage,
        vk::MemoryPropertyFlags memory_usage = vk::MemoryPropertyFlagBits::eDeviceLocal) const;

    template <typename T>
    UniqueMemoryBuffer MakeBuffer(
        const vk::CommandBuffer& cmd_buffer, const std::vector<T>& data, vk::BufferUsageFlags usage,
        vk::MemoryPropertyFlags mem_properties = vk::MemoryPropertyFlagBits::eDeviceLocal) const
    {
        auto buffer_size = DataSize(data);
        // 1. Creates a host-visible and -coherent staging buffer for data transfer.
        UniqueMemoryBuffer staging_buffer = MakeBuffer(
            buffer_size, vk::BufferUsageFlagBits::eTransferSrc,
            vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent);

        // 2. Copies the data to memory.
        {
            void* mapped = device_.mapMemory(staging_buffer.memory, /*offset=*/0, buffer_size);
            memcpy(mapped, data.data(), buffer_size);
            device_.unmapMemory(staging_buffer.memory);
        }

        // 3. Creates the result buffer.
        auto result_buffer = MakeBuffer(buffer_size, usage | vk::BufferUsageFlagBits::eTransferDst);

        // 4. Copies the data from staging buffer to the result buffer.
        cmd_buffer.copyBuffer(
            staging_buffer.handle, result_buffer.handle,
            vk::BufferCopy().setSize(buffer_size).setSrcOffset(0).setDstOffset(0));

        // 5. Cleanup: destroys the staging buffer.
        staging_buffers_.push_back(staging_buffer);

        return result_buffer;
    }

    // Images
    // --------------------------------------------------------------------------------------------

    UniqueMemoryImage MakeImage(const vk::ImageCreateInfo& image_ci,
                                vk::MemoryPropertyFlags = vk::MemoryPropertyFlagBits::eDeviceLocal);

    UniqueMemoryImage MakeImage(const vk::CommandBuffer& cmd_buffer, size_t size, const void* data,
                                const vk::ImageCreateInfo& image_ci,
                                const vk::ImageLayout = vk::ImageLayout::eShaderReadOnlyOptimal);

    // Textures
    // --------------------------------------------------------------------------------------------
    UniqueMemoryTexture MakeTexture(const UniqueMemoryImage&       image,
                                    const vk::ImageViewCreateInfo& image_view_ci,
                                    const vk::SamplerCreateInfo&   sampler_ci);

    UniqueMemoryTexture MakeTexture(const UniqueMemoryImage&       image,
                                    const vk::ImageViewCreateInfo& image_view_ci);


    u32 GetMemoryTypeIndex(u32 type_bits, vk::MemoryPropertyFlags desired_properties) const;

  private:
    vk::Device                              device_;
    vk::PhysicalDevice                      gpu_;
    mutable std::vector<UniqueMemoryBuffer> staging_buffers_;
};


}  // namespace vkpbr


#endif  // !NVCOPY_VK_MEMORY_H_
