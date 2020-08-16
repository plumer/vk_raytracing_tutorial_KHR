#ifndef NVCOPY_VK_UTILS_H_
#define NVCOPY_VK_UTILS_H_

#include <glm/glm.hpp>
#include <vector>
#include <vulkan/vulkan.hpp>

#include "types.h"
#include "vk_memory.h"

namespace vkpbr {

template<typename BitType>
bool FlagsMatch(vk::Flags<BitType> candidate, vk::Flags<BitType> desired_flags)
{
    return (candidate & desired_flags) == desired_flags;
}
template<typename BitType>
bool FlagsMatch(vk::Flags<BitType> candidate, BitType desired_flags)
{
    return FlagsMatch(candidate, vk::Flags<BitType>(desired_flags));
}

class CommandPool
{
  public:
    CommandPool(const vk::Device& device, u32 queue_family_index,
                vk::CommandPoolCreateFlags flags = vk::CommandPoolCreateFlagBits::eTransient,
                vk::Queue                  default_queue = nullptr);
    ~CommandPool();

    CommandPool(CommandPool const&) = delete;
    CommandPool& operator=(CommandPool const&) = delete;

    vk::CommandBuffer MakeCmdBuffer(
        vk::CommandBufferLevel level = vk::CommandBufferLevel::ePrimary, bool begin = true,
        vk::CommandBufferUsageFlags usage = vk::CommandBufferUsageFlagBits::eOneTimeSubmit,
        const vk::CommandBufferInheritanceInfo* p_inheritance_info = nullptr);

    void SubmitAndWait(vk::ArrayProxy<const vk::CommandBuffer> cmds, vk::Queue queue = nullptr);

  private:
    const vk::Device& device_;
    const vk::Queue   default_queue_;
    //u32               queue_index_ = 0;

    vk::CommandPool cmd_pool_;
};

class DescriptorSetBindings
{
  public:
    DescriptorSetBindings() = default;

    // Wrapper methods from std::vector<T>. `data()` returns pointer-to-const for now.
    void                                  clear() { bindings_.clear(); }
    bool                                  empty() const { return bindings_.empty(); }
    size_t                                size() const { return bindings_.size(); }
    const vk::DescriptorSetLayoutBinding* data() const { return bindings_.data(); }

    void AddBinding(u32 binding_index, vk::DescriptorType type, u32 count,
                    vk::ShaderStageFlags stage_flags,
                    const vk::Sampler*   p_immutable_sampler = nullptr);
    void AddBinding(const vk::DescriptorSetLayoutBinding& binding) { bindings_.push_back(binding); }

    vk::DescriptorType GetType(u32 binding_index) const;
    u32                GetCount(u32 binding_index) const;

    vk::DescriptorSetLayout MakeLayout(vk::Device device,
                                       vk::DescriptorSetLayoutCreateFlags = {}) const;

    vk::DescriptorPool MakePool(vk::Device device, u32 max_sets = 1) const;

    // Methods that make DescriptorSet writes.

    vk::WriteDescriptorSet MakeGeneralDSWrite(vk::DescriptorSet set, u32 binding_index,
                                              u32 array_element = 0) const;

    vk::WriteDescriptorSet MakeWrite(vk::DescriptorSet set, u32 binding_index,
                                     const vk::DescriptorImageInfo* p_image_info,
                                     u32                            array_element = 0) const;
    vk::WriteDescriptorSet MakeWrite(vk::DescriptorSet set, u32 binding_index,
                                     const vk::DescriptorBufferInfo* p_buffer_info,
                                     u32                             array_element = 0) const;
    vk::WriteDescriptorSet MakeWrite(vk::DescriptorSet set, u32 binding_index,
                                     const vk::BufferView* p_texel_buffer_view,
                                     u32                   array_element = 0) const;
    vk::WriteDescriptorSet MakeWrite(vk::DescriptorSet set, u32 binding_index,
                                     const vk::WriteDescriptorSetAccelerationStructureKHR* p_accel,
                                     u32 array_element = 0) const;
    vk::WriteDescriptorSet MakeWrite(
        vk::DescriptorSet set, u32 binding_index,
        const vk::WriteDescriptorSetInlineUniformBlockEXT* p_inline_uniform,
        u32                                                array_element = 0) const;

    // Methods that make DescriptorSet Writers that updates an array of objects.

    // Makes a descriptor set writer with the descriptor count equal to what is recorded.
    vk::WriteDescriptorSet MakeGeneralDSWriteArray(vk::DescriptorSet set, u32 binding_index) const;

    vk::WriteDescriptorSet MakeWriteArray(vk::DescriptorSet set, u32 binding_index,
                                          const vk::DescriptorImageInfo* p_image_info) const;
    vk::WriteDescriptorSet MakeWriteArray(vk::DescriptorSet set, u32 binding_index,
                                          const vk::DescriptorBufferInfo* p_buffer_info) const;
    vk::WriteDescriptorSet MakeWriteArray(vk::DescriptorSet set, u32 binding_index,
                                          const vk::BufferView* p_texel_buffer_view) const;
    vk::WriteDescriptorSet MakeWriteArray(
        vk::DescriptorSet set, u32 binding_index,
        const vk::WriteDescriptorSetAccelerationStructureKHR* p_accel) const;
    vk::WriteDescriptorSet MakeWriteArray(
        vk::DescriptorSet set, u32 binding_index,
        const vk::WriteDescriptorSetInlineUniformBlockEXT* p_inline_uniform) const;

    // An invalid descriptor type within the scope of enum class vk::DescriptorType.
    // By value it is equal to VK_DESCRIPTOR_TYPE_MAX_ENUM.
    static constexpr vk::DescriptorType kInvalidType =
        static_cast<vk::DescriptorType>(VK_DESCRIPTOR_TYPE_MAX_ENUM);

  private:
    std::vector<vk::DescriptorSetLayoutBinding> bindings_;
};


class RaytracingBuilder
{
  public:
    // Type Aliases.
    using AS                       = vk::AccelerationStructureKHR;
    using BuildASFlags             = vk::BuildAccelerationStructureFlagsKHR;
    using BuildASFlagBits          = vk::BuildAccelerationStructureFlagBitsKHR;
    using ASCreateGeometryTypeInfo = vk::AccelerationStructureCreateGeometryTypeInfoKHR;
    using ASGeometry               = vk::AccelerationStructureGeometryKHR;
    using ASBuildOffsetInfo        = vk::AccelerationStructureBuildOffsetInfoKHR;

    RaytracingBuilder(RaytracingBuilder const&) = delete;
    RaytracingBuilder& operator=(const RaytracingBuilder&) = delete;

    RaytracingBuilder() = default;

    struct Blas {
        UniqueMemoryAccelStruct accel_struct;
        BuildASFlags            flags;

        std::vector<ASCreateGeometryTypeInfo> as_create_geometry_info;
        std::vector<ASGeometry>               as_geometry;
        std::vector<ASBuildOffsetInfo>        as_build_offset_info;
    };
    struct Instance {
        u32                          blas_id      = 0;
        u32                          instance_id  = 0;
        u32                          hit_group_id = 0;
        u32                          mask         = 0xFF;
        vk::GeometryInstanceFlagsKHR flags =
            vk::GeometryInstanceFlagBitsKHR::eTriangleFacingCullDisable;
        glm::mat4 transform = glm::mat4(1.0f);
    };

    void Setup(const vk::Device& device, const UniqueMemoryAllocator* allocator, u32 queue_index);

    void Destroy();

    void BuildBlas(const std::vector<Blas>& blases,
                   BuildASFlags             flags = BuildASFlagBits::ePreferFastTrace);

    vk::AccelerationStructureInstanceKHR InstanceToVkGeometryInstanceKHR(const Instance &instance);

    void BuildTlas(const std::vector<Instance>& instances,
                   BuildASFlags                 flags = BuildASFlagBits::ePreferFastTrace);

    // Refits the TLAS using new instance matrices
    void UpdateTlasMatrices(const std::vector<Instance>& instances);

    // Refits the BLAS from updated buffers.
    void UpdateBlas(u32 blas_index);

    vk::AccelerationStructureKHR AccelStruct() const { return tlas_.accel_struct.handle; }


  private:
    struct Tlas {
        UniqueMemoryAccelStruct accel_struct;
        vk::AccelerationStructureCreateInfoKHR accel_struct_ci;
        vk::BuildAccelerationStructureFlagsKHR flags;
    };

    std::vector<Blas> blases_;
    Tlas              tlas_;

    UniqueMemoryBuffer instance_buffer_;

    vk::Device                   device_;
    const UniqueMemoryAllocator* allocator_ = nullptr;
    u32                          queue_index_ = 0;
};

}  // namespace vkpbr

#endif  // !NVCOPY_VK_UTILS_H_
