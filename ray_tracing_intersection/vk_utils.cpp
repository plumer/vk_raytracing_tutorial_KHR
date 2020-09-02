#include "vk_utils.h"

#include "nvvk/extensions_vk.hpp"

#include "logging.h"
#include <set>

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
    std::vector<vk::SubpassDependency>  subpass_dependencies;

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

VkBool32 DebugMessengerCallback(VkDebugUtilsMessageSeverityFlagBitsEXT      msg_severity,
                                VkDebugUtilsMessageTypeFlagsEXT             msg_type,
                                const VkDebugUtilsMessengerCallbackDataEXT* callback_data,
                                void*                                       user_data)
{
    auto severity = vk::DebugUtilsMessageSeverityFlagBitsEXT{msg_severity};
    auto type     = vk::DebugUtilsMessageTypeFlagsEXT{msg_type};
    const vk::DebugUtilsMessengerCallbackDataEXT& cb_data{*callback_data};

    LogSeverity level = LogSeverity::kInfo;
    switch (severity) {
        case vk::DebugUtilsMessageSeverityFlagBitsEXT::eVerbose:
        case vk::DebugUtilsMessageSeverityFlagBitsEXT::eInfo:
            level = LogSeverity::kInfo;
            break;
        case vk::DebugUtilsMessageSeverityFlagBitsEXT::eWarning:
            level = LogSeverity::kWarning;
            break;
        case vk::DebugUtilsMessageSeverityFlagBitsEXT::eError:
            level = LogSeverity::kError;
            break;
    }
    Logger(level, __FILE__, __LINE__)
        << callback_data->pMessageIdName << "-->" << callback_data->pMessage;

    if (callback_data->objectCount > 0) {
        for (u32 object = 0; object < callback_data->objectCount; ++object) {
            std::string obj_type = vk::to_string(cb_data.pObjects[object].objectType);
            LOG(INFO) << " Object[" << object << "] - Type " << obj_type << ", Value "
                      << (void*)cb_data.pObjects[object].objectHandle << ", Name \'"
                      << cb_data.pObjects[object].pObjectName << '\'';
        }
    }
    if (callback_data->cmdBufLabelCount > 0) {
        static thread_local char buffer[256];
        for (u32 label = 0; label < cb_data.cmdBufLabelCount; ++label) {
            auto name  = cb_data.pCmdBufLabels[label].pLabelName;
            auto color = cb_data.pCmdBufLabels[label].color;
            memset(buffer, 0, sizeof(buffer));
            sprintf(buffer, "{ %.4f, %.4f, %.4f, %.4f}", color[0], color[1], color[2], color[3]);
            LOG(INFO) << " Label[" << label << "] - name " << buffer;
        }
    }

    return VK_FALSE;
}

ContextCreateInfo::ContextCreateInfo(bool use_validation)
{
#ifdef _DEBUG
    instance_extensions.emplace_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME, /*optional = */ true);
    if (use_validation)
        instance_layers.emplace_back("VK_LAYER_KHRONOS_validation", /*optional = */ true);
#endif  // _DEBUG
}

void ContextCreateInfo::SetVersion(int major, int minor)
{
    CHECK_GE(major, 1);
    CHECK_GE(minor, 1);
    api_major = major;
    api_minor = minor;
}

void ContextCreateInfo::AddInstanceExtension(const char* name, bool optional)
{
    instance_extensions.emplace_back(name, optional);
}

void ContextCreateInfo::AddInstanceExtension(std::vector<const char*> names, bool optional)
{
    for (const char* name : names)
        instance_extensions.emplace_back(name, optional);
}

void ContextCreateInfo::AddInstanceLayer(const char* name, bool optional)
{
    instance_layers.emplace_back(name, optional);
}

void ContextCreateInfo::AddInstanceLayer(std::vector<const char*> names, bool optional)
{
    for (const char* name : names)
        instance_layers.emplace_back(name, optional);
}

void ContextCreateInfo::AddDeviceExtension(const char* name, bool optional,
                                                  void* p_feature_struct)
{
    device_extensions.emplace_back(name, optional, p_feature_struct);
}

void ContextCreateInfo::AddDeviceExtension(std::vector<const char*> names, bool optional,
                                                  std::vector<void*> p_features)
{
    if (p_features.empty()) {
        for (const char* name : names)
            device_extensions.emplace_back(name, optional);
    } else {
        CHECK_EQ(names.size(), p_features.size());
        for (size_t i = 0; i < names.size(); ++i) {
            device_extensions.emplace_back(names[i], optional, p_features[i]);
        }
    }
}

void ContextCreateInfo::RemoveInstanceExtension(const char* name)
{
    std::remove_if(instance_extensions.begin(), instance_extensions.end(),
                   [name](const Entry& e) { return std::string_view(name) == e.name; });
}

void ContextCreateInfo::RemoveInstanceLayer(const char* name)
{
    std::remove_if(instance_layers.begin(), instance_layers.end(),
                   [name](const Entry& e) { return std::string_view(name) == e.name; });
}

void ContextCreateInfo::RemoveDeviceExtension(const char* name)
{
    std::remove_if(device_extensions.begin(), device_extensions.end(),
                   [name](const Entry& e) { return std::string_view(name) == e.name; });
}

namespace vkgpu {
struct Features11Old {
    vk::PhysicalDeviceMultiviewFeatures              multiview;
    vk::PhysicalDevice16BitStorageFeatures           u16_storage;
    vk::PhysicalDeviceSamplerYcbcrConversionFeatures sampler_ycbcr_conversion;
    vk::PhysicalDeviceProtectedMemoryFeatures        protected_memory;
    vk::PhysicalDeviceShaderDrawParameterFeatures    draw_parameters;
    vk::PhysicalDeviceVariablePointerFeatures        variable_pointers;

    Features11Old()
    {
        multiview.pNext                = &u16_storage;
        u16_storage.pNext              = &sampler_ycbcr_conversion;
        sampler_ycbcr_conversion.pNext = &protected_memory;
        protected_memory.pNext         = &draw_parameters;
        draw_parameters.pNext          = &variable_pointers;
    }

    void Read(const vk::PhysicalDeviceVulkan11Features& features_11)
    {
        multiview.multiview                   = features_11.multiview;
        multiview.multiviewGeometryShader     = features_11.multiviewGeometryShader;
        multiview.multiviewTessellationShader = features_11.multiviewTessellationShader;

        u16_storage.storageBuffer16BitAccess = features_11.storageBuffer16BitAccess;
        u16_storage.storageInputOutput16     = features_11.storageInputOutput16;
        u16_storage.storagePushConstant16    = features_11.storagePushConstant16;
        u16_storage.uniformAndStorageBuffer16BitAccess =
            features_11.uniformAndStorageBuffer16BitAccess;

        sampler_ycbcr_conversion.samplerYcbcrConversion = features_11.samplerYcbcrConversion;

        protected_memory.protectedMemory = features_11.protectedMemory;

        draw_parameters.shaderDrawParameters = features_11.shaderDrawParameters;

        variable_pointers.variablePointers              = features_11.variablePointers;
        variable_pointers.variablePointersStorageBuffer = features_11.variablePointersStorageBuffer;
    }

    void Write(vk::PhysicalDeviceVulkan11Features* features_11)
    {
        features_11->multiview                   = multiview.multiview;
        features_11->multiviewGeometryShader     = multiview.multiviewGeometryShader;
        features_11->multiviewTessellationShader = multiview.multiviewTessellationShader;

        features_11->storageBuffer16BitAccess = u16_storage.storageBuffer16BitAccess;
        features_11->storageInputOutput16     = u16_storage.storageInputOutput16;
        features_11->storagePushConstant16    = u16_storage.storagePushConstant16;
        features_11->uniformAndStorageBuffer16BitAccess =
            u16_storage.uniformAndStorageBuffer16BitAccess;

        features_11->samplerYcbcrConversion = sampler_ycbcr_conversion.samplerYcbcrConversion;

        features_11->protectedMemory = protected_memory.protectedMemory;

        features_11->shaderDrawParameters = draw_parameters.shaderDrawParameters;

        features_11->variablePointers = variable_pointers.variablePointers;
        features_11->variablePointersStorageBuffer =
            variable_pointers.variablePointersStorageBuffer;
    }
};

struct Properties11Old {
    vk::PhysicalDeviceMaintenance3Properties    maintenance3;
    vk::PhysicalDeviceIDProperties              device_id;
    vk::PhysicalDeviceMultiviewProperties       multiview;
    vk::PhysicalDeviceProtectedMemoryProperties protected_memory;
    vk::PhysicalDevicePointClippingProperties   point_clipping;
    vk::PhysicalDeviceSubgroupProperties        subgroup;

    Properties11Old()
    {
        maintenance3.pNext     = &device_id;
        device_id.pNext        = &multiview;
        multiview.pNext        = &protected_memory;
        protected_memory.pNext = &point_clipping;
        point_clipping.pNext   = &subgroup;
    }

    void Write(vk::PhysicalDeviceVulkan11Properties& properties_11)
    {
        memcpy(properties_11.deviceLUID, device_id.deviceLUID, sizeof(properties_11.deviceLUID));
        memcpy(properties_11.deviceUUID, device_id.deviceUUID, sizeof(properties_11.deviceUUID));
        memcpy(properties_11.driverUUID, device_id.driverUUID, sizeof(properties_11.driverUUID));
        properties_11.deviceLUIDValid = device_id.deviceLUIDValid;
        properties_11.deviceNodeMask  = device_id.deviceNodeMask;

        properties_11.subgroupSize                      = subgroup.subgroupSize;
        properties_11.subgroupSupportedStages           = subgroup.supportedStages;
        properties_11.subgroupSupportedOperations       = subgroup.supportedOperations;
        properties_11.subgroupQuadOperationsInAllStages = subgroup.quadOperationsInAllStages;

        properties_11.pointClippingBehavior = point_clipping.pointClippingBehavior;

        properties_11.maxMultiviewViewCount     = multiview.maxMultiviewViewCount;
        properties_11.maxMultiviewInstanceIndex = multiview.maxMultiviewInstanceIndex;

        properties_11.protectedNoFault = protected_memory.protectedNoFault;

        properties_11.maxPerSetDescriptors    = maintenance3.maxPerSetDescriptors;
        properties_11.maxMemoryAllocationSize = maintenance3.maxMemoryAllocationSize;
    }
};
}  // namespace vkgpu

vk::DispatchLoaderStatic Context::static_loader_ = {};

bool Context::Init(const ContextCreateInfo& context_ci)
{
    if (!InitInstance(context_ci))
        return false;

    auto compatible_devices = GetCompatibleDevices(context_ci);
    CHECK(!compatible_devices.empty()) << "no compatible GPUs found";

    return InitDevice(compatible_devices[context_ci.compatible_device_index], context_ci);
}

void Context::DeInit()
{
    if (device_) {
        device_.waitIdle();
    }
    device_.destroy(nullptr);
    device_ = nullptr;
    if (fp_destroyDebugUtilsMessengerEXT)
        fp_destroyDebugUtilsMessengerEXT(instance_, debug_messenger_, nullptr);

    if (instance_) {
        instance_.destroy();
        instance_ = nullptr;
    }

    used_instance_extensions_.clear();
    used_instance_layers_.clear();
    used_device_extensions_.clear();

    fp_createDebugUtilsMessengerEXT_ = nullptr;
    fp_destroyDebugUtilsMessengerEXT = nullptr;
    debug_messenger_                 = nullptr;

    reset_VK_EXTENSION_SUBSET();
}

bool Context::InitInstance(const ContextCreateInfo& context_ci)
{
    vk::DynamicLoader dl;
    PFN_vkGetInstanceProcAddr fp_vkGetInstanceProcAddr =
        dl.getProcAddress<PFN_vkGetInstanceProcAddr>("vkGetInstanceProcAddr");
    VULKAN_HPP_DEFAULT_DISPATCHER.init(fp_vkGetInstanceProcAddr);

    auto application_info =
        vk::ApplicationInfo()
            .setPApplicationName(context_ci.app_title)
            .setPEngineName(context_ci.app_engine_name)
            .setApiVersion(VK_MAKE_VERSION(context_ci.api_major, context_ci.api_minor, 0));
    u32 count = 0;
    if (context_ci.verbose_used) {
        u32 version = vk::enumerateInstanceVersion(static_loader_);
        LOG(INFO) << "Vulkan version: ";
        printf(" - available:  %d.%d.%d\n", VK_VERSION_MAJOR(version), VK_VERSION_MINOR(version),
               VK_VERSION_PATCH(version));
        printf(" - requesting: %d.%d.%d\n", context_ci.api_major, context_ci.api_minor, 0);
    }

    {
        auto layer_properties = GetInstanceLayers();

        if (FillFilterNameArray(&used_instance_layers_, layer_properties,
                                context_ci.instance_layers)
            != vk::Result::eSuccess) {
            return false;
        }
        if (context_ci.verbose_available) {
            LOG(INFO) << "-------------------------------";
            LOG(INFO) << "Available Instance Layers: ";
            for (auto prop : layer_properties) {
                printf(" %s (ver. %x %x) : %s\n", prop.layerName, prop.specVersion,
                       prop.implementationVersion, prop.description);
            }
        }
    }
    {  // Gets all extensions
        auto extension_properties = GetInstanceExtensions();

        std::vector<void*> features_structs;
        if (FillFilterNameArray(&used_instance_extensions_, extension_properties,
                                context_ci.instance_extensions, features_structs)
            != vk::Result::eSuccess)
            return false;

        if (context_ci.verbose_available) {
            LOG(INFO) << "\nAvailable Instance Extensions :";
            for (auto prop : extension_properties) {
                printf(" %s (ver. %d)\n", prop.extensionName, prop.specVersion);
            }
        }
    }
    if (context_ci.verbose_used) {
        LOG(INFO) << "-----------------\nUsed Instance Layers: ";
        for (auto layer : used_instance_layers_) {
            LOG(INFO) << " " << layer;
        }
        LOG(INFO) << "-----------------\nUsed Instance Extensions: ";
        for (auto ext : used_instance_extensions_) {
            LOG(INFO) << " " << ext;
        }
    }

    auto instance_ci = vk::InstanceCreateInfo()
                           .setPApplicationInfo(&application_info)
                           .setEnabledExtensionCount(cast_u32(used_instance_extensions_.size()))
                           .setPpEnabledExtensionNames(used_instance_extensions_.data())
                           .setEnabledLayerCount(cast_u32(used_instance_layers_.size()))
                           .setPpEnabledLayerNames(used_instance_layers_.data());

    instance_ = vk::createInstance(instance_ci, nullptr, static_loader_);
    VULKAN_HPP_DEFAULT_DISPATCHER.init(instance_);

    for (const auto& ext : used_instance_extensions_) {
        if (std::string_view(ext) == VK_EXT_DEBUG_UTILS_EXTENSION_NAME) {
            InitDebugUtils();
            break;
        }
    }

    return true;
}

bool Context::InitDevice(u32 device_index, const ContextCreateInfo& context_ci)
{
    CHECK(instance_);

    vk::PhysicalDeviceGroupProperties gpu_group_properties;
    if (context_ci.use_device_groups) {
        auto groups = GetGpuGroups();
        CHECK_LT(device_index, cast_u32(groups.size()));
        gpu_group_properties = groups[device_index];
        gpu_                 = gpu_group_properties.physicalDevices[0];
    } else {
        auto gpus = GetGpus();
        CHECK_LT(device_index, cast_u32(gpus.size()));
        gpu_ = gpus[device_index];
    }

    InitGpuInfo(&gpu_info_, gpu_, context_ci.api_major, context_ci.api_minor);

    auto                 features2 = vk::PhysicalDeviceFeatures2();
    vkgpu::Features11Old features_11_old;
    features2.features = gpu_info_.features_100;
    features_11_old.Read(gpu_info_.features_110);

    if (context_ci.api_major == 1 && context_ci.api_minor == 1) {
        features2.pNext = &features_11_old.multiview;
    } else if (context_ci.api_major == 1 && context_ci.api_minor >= 2) {
        features2.pNext              = &gpu_info_.features_110;
        gpu_info_.features_110.pNext = &gpu_info_.features_120;
    }

    std::vector<vk::DeviceQueueCreateInfo> queue_cis;
    std::vector<float>                     priorities;
    std::vector<void*>                     feature_structures;

    bool queue_family_general_purpose = false;
    {
        for (auto& prop : gpu_info_.queue_properties) {
            if (FlagsMatch(prop.queueFlags, vk::QueueFlagBits::eGraphics
                                                | vk::QueueFlagBits::eCompute
                                                | vk::QueueFlagBits::eTransfer))
                queue_family_general_purpose = true;
            if (prop.queueCount > priorities.size())
                priorities.resize(prop.queueCount, 1.0f);  // Sets all priorities equal.
        }
        for (u32 i = 0; i < gpu_info_.queue_properties.size(); ++i) {
            auto queue_ci = vk::DeviceQueueCreateInfo()
                                .setQueueFamilyIndex(i)
                                .setQueueCount(gpu_info_.queue_properties[i].queueCount)
                                .setPQueuePriorities(priorities.data());
            queue_cis.push_back(queue_ci);
        }
    }

    LOG_IF(WARNING, !queue_family_general_purpose) << "couldn't find queue that supports graphics, "
                                                      "compute and transfer at the same time";

    auto device_ci = vk::DeviceCreateInfo()
                         .setQueueCreateInfoCount(cast_u32(queue_cis.size()))
                         .setPQueueCreateInfos(queue_cis.data());

    auto extension_props = GetGpuExtensions(gpu_);

    if (context_ci.verbose_available) {
        LOG(INFO) << "\nAvailable GPU Extensions: ";
        for (auto& prop : extension_props) {
            printf(" %s (ver. %d)\n", prop.extensionName, prop.specVersion);
        }
    }

    if (FillFilterNameArray(&used_device_extensions_, extension_props, context_ci.device_extensions,
                            feature_structures)
        != vk::Result::eSuccess) {
        DeInit();
        return false;
    }

    if (context_ci.verbose_used) {
        LOG(INFO) << "\nUsed GPU Extensions :\n";
        for (auto& ext : used_device_extensions_) {
            LOG(INFO) << "  " << ext;
        }
    }

    struct ExtensionHeader {
        vk::StructureType struct_type;
        void*             p_next;
    };
    // Uses feature_2 chain to append extensions
    if (!feature_structures.empty()) {
        for (size_t i = 0; i < feature_structures.size(); ++i) {
            auto* header = reinterpret_cast<ExtensionHeader*>(feature_structures[i]);
            CHECK(header);
            header->p_next =
                (i < feature_structures.size() - 1) ? feature_structures[i + 1] : nullptr;
        }

        // Appends to the end of current feature_2 struct.
        ExtensionHeader* last_core_feature = reinterpret_cast<ExtensionHeader*>(&features2);
        while (last_core_feature->p_next != nullptr)
            last_core_feature = reinterpret_cast<ExtensionHeader*>(last_core_feature->p_next);
        last_core_feature->p_next = feature_structures[0];

        // Queries GPU features support.
        gpu_.getFeatures2(&features2);
    }

    if (context_ci.disable_robust_buffer_access) {
        features2.features.robustBufferAccess = VK_FALSE;
    }

    device_ci.enabledExtensionCount   = cast_u32(used_device_extensions_.size());
    device_ci.ppEnabledExtensionNames = used_device_extensions_.data();

    device_ci.pEnabledFeatures = nullptr;
    device_ci.pNext            = &features2;

    auto device_group_ci = vk::DeviceGroupDeviceCreateInfo();
    if (context_ci.use_device_groups) {
        device_group_ci.pNext               = device_ci.pNext;
        device_group_ci.physicalDeviceCount = cast_u32(gpu_group_properties.physicalDeviceCount);
        device_group_ci.pPhysicalDevices    = gpu_group_properties.physicalDevices;
        device_group_ci.pNext               = &device_group_ci;
    }

    ExtensionHeader* device_create_chain = nullptr;
    if (context_ci.device_create_info_ext) {
        device_create_chain = reinterpret_cast<ExtensionHeader*>(context_ci.device_create_info_ext);
        while (device_create_chain->p_next != nullptr)
            device_create_chain = reinterpret_cast<ExtensionHeader*>(device_create_chain->p_next);
        // Overrides last of external chain.
        device_create_chain->p_next = const_cast<void*>(device_ci.pNext);
        device_ci.pNext             = context_ci.device_create_info_ext;
    }
    device_ = gpu_.createDevice(device_ci);

    if (device_create_chain)
        device_create_chain->p_next = nullptr;

    load_VK_EXTENSION_SUBSET(instance_, vkGetInstanceProcAddr, device_, vkGetDeviceProcAddr);

    VULKAN_HPP_DEFAULT_DISPATCHER.init(device_);

    // Gets default queues.
    u32 queue_family_index = 0;
    for (auto& queue_prop : gpu_info_.queue_properties) {
        if (FlagsMatch(queue_prop.queueFlags, vk::QueueFlagBits::eGraphics
                                                  | vk::QueueFlagBits::eCompute
                                                  | vk::QueueFlagBits::eTransfer)) {
            queue_graphics_compute_transfer_ = {device_.getQueue(queue_family_index, 0),
                                                queue_family_index};
        } else if (FlagsMatch(queue_prop.queueFlags, vk::QueueFlagBits::eTransfer)) {
            queue_transfer_ = {device_.getQueue(queue_family_index, 0), queue_family_index};
        } else if (FlagsMatch(queue_prop.queueFlags, vk::QueueFlagBits::eCompute)) {

            queue_compute_ = {device_.getQueue(queue_family_index, 0), queue_family_index};
        }
        ++queue_family_index;
    }
    return true;
}

std::vector<u32> Context::GetCompatibleDevices(const ContextCreateInfo& context_ci) const
{
    CHECK(instance_);
    std::vector<u32>                               compatible_devices;
    std::vector<vk::PhysicalDeviceGroupProperties> groups;
    std::vector<vk::PhysicalDevice>                gpus;

    u32 num_elements = 0;
    if (context_ci.use_device_groups) {
        groups       = GetGpuGroups();
        num_elements = cast_u32(groups.size());
    } else {
        gpus         = GetGpus();
        num_elements = cast_u32(gpus.size());
    }

    if (context_ci.verbose_compatible_devices) {
        LOG(INFO) << "Compatible devices:";
    }

    u32 compatible = 0;
    for (u32 element_id = 0; element_id < num_elements; ++element_id) {
        auto gpu =
            context_ci.use_device_groups ? groups[element_id].physicalDevices[0] : gpus[element_id];
        // All physical devices in a group are identical.
        if (HasMandatoryExtensions(gpu, context_ci, context_ci.verbose_compatible_devices)) {
            compatible_devices.push_back(element_id);
            if (context_ci.verbose_compatible_devices) {
                LOG(INFO) << compatible << ": " << gpu.getProperties().deviceName;
                ++compatible;
            }
        } else if (context_ci.verbose_compatible_devices) {
            LOG(WARNING) << "Skipping GPU " << gpu.getProperties().deviceName;
        }
    }

    if (context_ci.verbose_compatible_devices) {
        LOG(INFO) << "GPU found : " << compatible;
        if (compatible == 0)
            LOG_ERROR << "No compatible GPUs";
    }

    return compatible_devices;
}

bool Context::HasMandatoryExtensions(vk::PhysicalDevice gpu, const ContextCreateInfo& context_ci,
                                     bool verbose) const
{
    auto extension_properties = gpu.enumerateDeviceExtensionProperties(nullptr, static_loader_);
    return CheckEntryArray(extension_properties, context_ci.device_extensions, verbose);
}

bool Context::SetGCTQueueWithPresent(vk::SurfaceKHR surface)
{
    auto bits =
        vk::QueueFlagBits::eGraphics | vk::QueueFlagBits::eCompute | vk::QueueFlagBits::eTransfer;

    for (u32 qf_index = 0; qf_index < cast_u32(gpu_info_.queue_properties.size()); ++qf_index) {
        vk::Bool32 supports_present = gpu_.getSurfaceSupportKHR(qf_index, surface);

        if (supports_present == VK_TRUE
            && FlagsMatch(gpu_info_.queue_properties[qf_index].queueFlags, bits)) {
            queue_graphics_compute_transfer_ = {device_.getQueue(qf_index, 0), qf_index};
            return true;
        }
    }
    return false;
}

u32 Context::GetQueueFamily(vk::QueueFlags flags_supported, vk::QueueFlags flags_disabled,
                            vk::SurfaceKHR surface)
{
    LOG(FATAL) << "unused function";
    return u32();
}

bool Context::HasDeviceExtension(const char* name) const
{
    for (const auto& ext : used_instance_extensions_) {
        if (std::string_view(name) == ext)
            return true;
    }
    return false;
}

bool Context::HasInstanceExtension(const char* name) const
{
    for (const auto& ext : used_instance_extensions_) {
        if (std::string_view(name) == ext)
            return true;
    }
    return false;
}

void Context::InitDebugUtils()
{
    fp_createDebugUtilsMessengerEXT_ = reinterpret_cast<PFN_vkCreateDebugUtilsMessengerEXT>(
        instance_.getProcAddr("vkCreateDebugUtilsMessengerEXT", static_loader_));
    fp_destroyDebugUtilsMessengerEXT = reinterpret_cast<PFN_vkDestroyDebugUtilsMessengerEXT>(
        instance_.getProcAddr("vkDestroyDebugUtilsMessengerEXT", static_loader_));
    CHECK(fp_createDebugUtilsMessengerEXT_) << "debug utils creation not supported";
    CHECK(fp_destroyDebugUtilsMessengerEXT) << "debug utils destruction not supported";

    using Severity = vk::DebugUtilsMessageSeverityFlagBitsEXT;
    using MType    = vk::DebugUtilsMessageTypeFlagBitsEXT;
    VkDebugUtilsMessengerCreateInfoEXT debug_messenger_ci =
        vk::DebugUtilsMessengerCreateInfoEXT()
            .setMessageSeverity(Severity::eWarning | Severity::eError)
            .setMessageType(MType::eGeneral | MType::eValidation | MType::ePerformance)
            .setPfnUserCallback(DebugMessengerCallback)
            .setPUserData(nullptr);
    VkResult r = fp_createDebugUtilsMessengerEXT_(instance_, &debug_messenger_ci, nullptr,
                                                  &debug_messenger_);
    CHECK_EQ(r, VK_SUCCESS);
}

vk::Result Context::FillFilterNameArray(std::vector<const char*>*                    used,
                                        const std::vector<vk::LayerProperties>&      properties,
                                        const std::vector<ContextCreateInfo::Entry>& requested)
{
    for (const auto &entry : requested) {
        bool found = false;
        for (const auto& prop : properties) {
            if (std::string_view(entry.name) == prop.layerName) {
                found = true;
                break;
            }
        }
        if (!found && !entry.optional) {
            LOG(WARNING) << "vk::Result::eErrorExtensionNotPresent: " << entry.name;
            return vk::Result::eErrorExtensionNotPresent;
        }
        used->push_back(entry.name);
    }
    return vk::Result::eSuccess;
}

vk::Result Context::FillFilterNameArray(std::vector<const char*>*                    used,
                                        const std::vector<vk::ExtensionProperties>&  properties,
                                        const std::vector<ContextCreateInfo::Entry>& requested,
                                        std::vector<void*>& feature_structs)
{
    for (const auto& entry : requested) {
        bool found = false;
        for (const auto& prop : properties) {
            if (std::string_view(entry.name) == prop.extensionName
                && (entry.version == 0 || entry.version == prop.specVersion)) {
                found = true;
                break;
            }
        }
        if (found) {
            used->push_back(entry.name);
            if (entry.p_feature_struct)
                feature_structs.push_back(entry.p_feature_struct);
        } else if (!entry.optional) {
            LOG(WARNING) << "vk::Result::eErrorExtensionNotPresent: " << entry.name << " - "
                         << entry.version;
            return vk::Result::eErrorExtensionNotPresent;
        }
    }
    return vk::Result::eSuccess;
}

bool Context::CheckEntryArray(const std::vector<vk::ExtensionProperties>&  properties,
                              const std::vector<ContextCreateInfo::Entry>& requested,
                              bool                                         verbose) const
{
    std::set<std::string> supported_extensions;
    for (const auto& prop : properties)
        supported_extensions.insert(prop.extensionName);
    for (const auto& entry : requested) {
        // Checks if `entry` is in properties.
        auto iter = supported_extensions.find(entry.name);
        if (iter == supported_extensions.end() && entry.optional) {
            LOG_IF(WARNING, verbose) << "Can't locate mandatory extension " << entry.name;
            return false;
        }
    }
    return true;
}

void Context::InitGpuInfo(GpuInfo* info, vk::PhysicalDevice gpu, u32 version_major,
                          u32 version_minor)
{
    info->memory_properties = gpu.getMemoryProperties();
    info->queue_properties  = gpu.getQueueFamilyProperties();

    // For queries and device creation.
    vk::PhysicalDeviceFeatures2   features_2;
    vk::PhysicalDeviceProperties2 properties_2;
    vkgpu::Features11Old          features_110_old;
    vkgpu::Properties11Old        properties_110_old;

    if (version_major == 1 && version_minor == 1) {
        features_2.pNext   = &features_110_old.multiview;
        properties_2.pNext = &properties_110_old.maintenance3;
    } else if (version_major == 1 && version_minor >= 2) {
        features_2.pNext         = &info->features_110;
        info->features_110.pNext = &info->features_120;
        info->features_120.pNext = nullptr;

        info->properties_120.driverID                     = vk::DriverId::eNvidiaProprietary;
        info->properties_120.supportedDepthResolveModes   = vk::ResolveModeFlagBits::eMax;
        info->properties_120.supportedStencilResolveModes = vk::ResolveModeFlagBits::eMax;

        properties_2.pNext         = &info->properties_110;
        info->properties_110.pNext = &info->properties_120;
        info->properties_120.pNext = nullptr;
    }

    gpu.getFeatures2(&features_2);
    gpu.getProperties2(&properties_2);

    info->properties_100 = properties_2.properties;
    info->features_100   = features_2.features;

    if (version_major == 1 && version_minor == 1) {
        properties_110_old.Write(info->properties_110);
        features_110_old.Write(&info->features_110);
    }
}

}  // namespace vkpbr
