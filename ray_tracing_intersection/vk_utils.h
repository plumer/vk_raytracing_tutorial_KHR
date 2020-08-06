#ifndef NVCOPY_VK_UTILS_H_
#define NVCOPY_VK_UTILS_H_

#include <glm/glm.hpp>
#include <vector>
#include <vulkan/vulkan.hpp>

#include "types.h"

namespace vkpbr {

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

}  // namespace vkpbr

#endif  // !NVCOPY_VK_UTILS_H_
