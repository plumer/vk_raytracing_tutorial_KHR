#include "vk_utils.h"

#include "logging.h"

namespace {
using Binding   = vk::DescriptorSetLayoutBinding;
using Type      = vk::DescriptorType;
using Stage     = vk::ShaderStageFlags;
using StageBits = vk::ShaderStageFlagBits;
using Writer    = vk::WriteDescriptorSet;
}  // namespace

namespace vkpbr {

void DescriptorSetBindings::AddBinding(u32 binding, vk::DescriptorType type, u32 count,
                                       vk::ShaderStageFlags stage_flags,
                                       const vk::Sampler*   p_immutable_sampler) {
    bindings_.emplace_back(Binding()
                               .setBinding(binding)
                               .setDescriptorType(type)
                               .setDescriptorCount(count)
                               .setStageFlags(stage_flags)
                               .setPImmutableSamplers(p_immutable_sampler));
}

vk::DescriptorType DescriptorSetBindings::GetType(u32 binding_index) const {
    for (const auto& binding : bindings_) {
        if (binding.binding == binding_index)
            return binding.descriptorType;
    }
    return kInvalidType;
}

u32 DescriptorSetBindings::GetCount(u32 binding_index) const {
    for (const auto& binding : bindings_) {
        if (binding.binding == binding_index)
            return binding.descriptorCount;
    }
    LOG_FATAL << "binding index not found";
    return -1;
}


vk::DescriptorSetLayout DescriptorSetBindings::MakeLayout(
    vk::Device device, vk::DescriptorSetLayoutCreateFlags flags) const {
    auto ds_layout_ci =
        vk::DescriptorSetLayoutCreateInfo().setBindingCount(size()).setPBindings(data()).setFlags(
            flags);

    return device.createDescriptorSetLayout(ds_layout_ci);
}

vk::DescriptorPool DescriptorSetBindings::MakePool(vk::Device device, u32 max_sets) const {
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
                                                 u32 array_element) const {
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
                                        u32                            array_element) const {
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
                                        u32                             array_element) const {
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
                                        u32                   array_element) const {
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
    const vk::WriteDescriptorSetAccelerationStructureKHR* p_accel, u32 array_element) const {
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
    const vk::WriteDescriptorSetInlineUniformBlockEXT* p_inline_uniform, u32 array_element) const {
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
                                                                      u32 binding_index) const {
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
    vk::DescriptorSet set, u32 binding_index, const vk::DescriptorImageInfo* p_image_info) const {
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
    vk::DescriptorSet set, u32 binding_index, const vk::DescriptorBufferInfo* p_buffer_info) const {
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
    vk::DescriptorSet set, u32 binding_index, const vk::BufferView* p_texel_buffer_view) const {
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
    const vk::WriteDescriptorSetAccelerationStructureKHR* p_accel) const {
    auto ds_write = MakeGeneralDSWriteArray(set, binding_index);
    CHECK(ds_write.descriptorType == Type::eAccelerationStructureKHR);
    ds_write.pNext = p_accel;
    return ds_write;
}

vk::WriteDescriptorSet DescriptorSetBindings::MakeWriteArray(
    vk::DescriptorSet set, u32 binding_index,
    const vk::WriteDescriptorSetInlineUniformBlockEXT* p_inline_uniform) const {
    auto ds_write = MakeGeneralDSWriteArray(set, binding_index);
    CHECK(ds_write.descriptorType == Type::eInlineUniformBlockEXT);
    ds_write.pNext = p_inline_uniform;
    return ds_write;
}

}  // namespace vkpbr
