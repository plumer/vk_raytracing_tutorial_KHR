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
        LOG(FATAL) << "command buffer submit failed: " << e.what();
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
    // TODO: checks if all binding indices are distinct.
    auto ds_layout_ci = vk::DescriptorSetLayoutCreateInfo().setBindings(bindings_).setFlags(flags);

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
    auto pool_ci = vk::DescriptorPoolCreateInfo().setMaxSets(max_sets).setPoolSizes(pool_sizes);
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
    return MakeImage2DCreateInfo(size, format, usage, false);
}

vk::ImageCreateInfo MakeImage2DCreateInfo(const vk::Extent2D& size, vk::Format format,
                                          vk::ImageUsageFlags usage, bool use_mipmaps)
{
    return vk::ImageCreateInfo()
        .setImageType(vk::ImageType::e2D)
        .setFormat(format)
        .setSamples(vk::SampleCountFlagBits::e1)
        .setMipLevels(use_mipmaps ? MipLevels(size) : 1)
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
    auto severity = vk::DebugUtilsMessageSeverityFlagBitsEXT(msg_severity);
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
            const char * obj_name = cb_data.pObjects[object].pObjectName;
            if (!obj_name)
                obj_name = "";
            LOG(INFO) << " Object[" <</* int(object) <<*/ "] - Type " << obj_type << ", Value "
                      << (void*)cb_data.pObjects[object].objectHandle << ", Name \'"
                      << obj_name<< '\'';
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

// ContextCreateInfo, Context
// ------------------------------------------------------------------------------------------------

#ifdef USE_NV_CONTEXT

static std::string ObjectTypeToString(VkObjectType value)
{
    switch (value) {
        case VK_OBJECT_TYPE_UNKNOWN:
            return "Unknown";
        case VK_OBJECT_TYPE_INSTANCE:
            return "Instance";
        case VK_OBJECT_TYPE_PHYSICAL_DEVICE:
            return "PhysicalDevice";
        case VK_OBJECT_TYPE_DEVICE:
            return "Device";
        case VK_OBJECT_TYPE_QUEUE:
            return "Queue";
        case VK_OBJECT_TYPE_SEMAPHORE:
            return "Semaphore";
        case VK_OBJECT_TYPE_COMMAND_BUFFER:
            return "CommandBuffer";
        case VK_OBJECT_TYPE_FENCE:
            return "Fence";
        case VK_OBJECT_TYPE_DEVICE_MEMORY:
            return "DeviceMemory";
        case VK_OBJECT_TYPE_BUFFER:
            return "Buffer";
        case VK_OBJECT_TYPE_IMAGE:
            return "Image";
        case VK_OBJECT_TYPE_EVENT:
            return "Event";
        case VK_OBJECT_TYPE_QUERY_POOL:
            return "QueryPool";
        case VK_OBJECT_TYPE_BUFFER_VIEW:
            return "BufferView";
        case VK_OBJECT_TYPE_IMAGE_VIEW:
            return "ImageView";
        case VK_OBJECT_TYPE_SHADER_MODULE:
            return "ShaderModule";
        case VK_OBJECT_TYPE_PIPELINE_CACHE:
            return "PipelineCache";
        case VK_OBJECT_TYPE_PIPELINE_LAYOUT:
            return "PipelineLayout";
        case VK_OBJECT_TYPE_RENDER_PASS:
            return "RenderPass";
        case VK_OBJECT_TYPE_PIPELINE:
            return "Pipeline";
        case VK_OBJECT_TYPE_DESCRIPTOR_SET_LAYOUT:
            return "DescriptorSetLayout";
        case VK_OBJECT_TYPE_SAMPLER:
            return "Sampler";
        case VK_OBJECT_TYPE_DESCRIPTOR_POOL:
            return "DescriptorPool";
        case VK_OBJECT_TYPE_DESCRIPTOR_SET:
            return "DescriptorSet";
        case VK_OBJECT_TYPE_FRAMEBUFFER:
            return "Framebuffer";
        case VK_OBJECT_TYPE_COMMAND_POOL:
            return "CommandPool";
        case VK_OBJECT_TYPE_SAMPLER_YCBCR_CONVERSION:
            return "SamplerYcbcrConversion";
        case VK_OBJECT_TYPE_DESCRIPTOR_UPDATE_TEMPLATE:
            return "DescriptorUpdateTemplate";
        case VK_OBJECT_TYPE_SURFACE_KHR:
            return "SurfaceKHR";
        case VK_OBJECT_TYPE_SWAPCHAIN_KHR:
            return "SwapchainKHR";
        case VK_OBJECT_TYPE_DISPLAY_KHR:
            return "DisplayKHR";
        case VK_OBJECT_TYPE_DISPLAY_MODE_KHR:
            return "DisplayModeKHR";
        case VK_OBJECT_TYPE_DEBUG_REPORT_CALLBACK_EXT:
            return "DebugReportCallbackEXT";
#if VK_NV_device_generated_commands
        case VK_OBJECT_TYPE_INDIRECT_COMMANDS_LAYOUT_NV:
            return "IndirectCommandsLayoutNV";
#endif
        case VK_OBJECT_TYPE_DEBUG_UTILS_MESSENGER_EXT:
            return "DebugUtilsMessengerEXT";
        case VK_OBJECT_TYPE_VALIDATION_CACHE_EXT:
            return "ValidationCacheEXT";
#if VK_NV_ray_tracing
        case VK_OBJECT_TYPE_ACCELERATION_STRUCTURE_NV:
            return "AccelerationStructureNV";
#endif
        default:
            return "invalid";
    }
}


// Define a callback to capture the messages
VKAPI_ATTR VkBool32 VKAPI_CALL Context::debugMessengerCallback(
    VkDebugUtilsMessageSeverityFlagBitsEXT      messageSeverity,
    VkDebugUtilsMessageTypeFlagsEXT             messageType,
    const VkDebugUtilsMessengerCallbackDataEXT* callbackData, void* userData)
{
    Context* ctx = reinterpret_cast<Context*>(userData);

    if (ctx->m_dbgIgnoreMessages.find(callbackData->messageIdNumber)
        != ctx->m_dbgIgnoreMessages.end())
        return VK_FALSE;


    LogSeverity level = LogSeverity::kInfo;
    // repeating nvprintfLevel to help with breakpoints : so we can selectively break right after
    // the print
    if (messageSeverity & VK_DEBUG_UTILS_MESSAGE_SEVERITY_VERBOSE_BIT_EXT) {
        printf("VERBOSE: %s \n --> %s\n", callbackData->pMessageIdName,
                      callbackData->pMessage);
    } else if (messageSeverity & VK_DEBUG_UTILS_MESSAGE_SEVERITY_INFO_BIT_EXT) {
        printf("INFO: %s \n --> %s\n", callbackData->pMessageIdName,
                      callbackData->pMessage);
    } else if (messageSeverity & VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT) {
        level = LogSeverity::kWarning;
        printf("WARNING: %s \n --> %s\n", callbackData->pMessageIdName,
                      callbackData->pMessage);
    } else if (messageSeverity & VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT) {
        level = LogSeverity::kError;
        printf("ERROR: %s \n --> %s\n", callbackData->pMessageIdName,
                      callbackData->pMessage);
    } else if (messageType & VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT) {
        printf("GENERAL: %s \n --> %s\n", callbackData->pMessageIdName,
                      callbackData->pMessage);
    } else {
        printf("%s \n --> %s\n", callbackData->pMessageIdName,
                      callbackData->pMessage);
    }

    // this seems redundant with the info already in callbackData->pMessage

    // Don't bail out, but keep going.
    return VK_FALSE;
}


//--------------------------------------------------------------------------------------------------
// Create the Vulkan instance and then first compatible device based on \p info
//
bool Context::Init(const ContextCreateInfo& info)
{
    if (!InitInstance(info))
        return false;

    // Find all compatible devices
    auto compatibleDevices = GetCompatibleDevices(info);
    if (compatibleDevices.empty()) {
        assert(!"No compatible device found");
        return false;
    }

    // Use a compatible device
    return InitDevice(compatibleDevices[info.compatibleDeviceIndex], info);
}

//--------------------------------------------------------------------------------------------------
// Create the Vulkan instance
//
bool Context::InitInstance(const ContextCreateInfo& info)
{
    VkApplicationInfo applicationInfo{VK_STRUCTURE_TYPE_APPLICATION_INFO};
    applicationInfo.pApplicationName = info.appTitle;
    applicationInfo.pEngineName      = info.appEngine;
    applicationInfo.apiVersion       = VK_MAKE_VERSION(info.apiMajor, info.apiMinor, 0);

    uint32_t count = 0;

    if (info.verboseUsed) {
        uint32_t version;
        VkResult result = vkEnumerateInstanceVersion(&version);
        CHECK(result == VK_SUCCESS);
        LOG(INFO) << "_______________\n" << "Vulkan Version:\n";
        printf(" - available:  %d.%d.%d\n", VK_VERSION_MAJOR(version), VK_VERSION_MINOR(version),
             VK_VERSION_PATCH(version));
        printf(" - requesting: %d.%d.%d\n", info.apiMajor, info.apiMinor, 0);
    }

    {
        // Get all layers
        auto layerProperties = GetInstanceLayers();

        if (fillFilteredNameArray(m_usedInstanceLayers, layerProperties, info.instanceLayers)
            != VK_SUCCESS) {
            return false;
        }

        if (info.verboseAvailable) {
            LOG(INFO) << "___________________________\n" << "Available Instance Layers :\n";
            for (auto it : layerProperties) {
                printf("%s (v. %x %x) : %s\n", it.layerName, it.specVersion, it.implementationVersion,
                     it.description);
            }
        }
    }

    {
        // Get all extensions
        auto extensionProperties = GetInstanceExtensions();

        std::vector<void*> featureStructs;
        if (fillFilteredNameArray(m_usedInstanceExtensions, extensionProperties,
                                  info.instanceExtensions, featureStructs)
            != VK_SUCCESS) {
            return false;
        }

        if (info.verboseAvailable) {
            LOG(INFO) << "Available Instance Extensions :";
            for (auto it : extensionProperties) {
                LOG(INFO) << Format("%s (v. %d)\n", it.extensionName, it.specVersion);
            }
        }
    }


    if (info.verboseUsed) {
        LOG(INFO) << "______________________\n" << "Used Instance Layers :\n";
        for (auto it : m_usedInstanceLayers) {
            LOG(INFO) << it;
        }
        LOG(INFO) << "Used Instance Extensions :\n";
        for (auto it : m_usedInstanceExtensions) {
            LOG(INFO) << it;
        }
    }

    VkInstanceCreateInfo instanceCreateInfo{VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO};
    instanceCreateInfo.pApplicationInfo = &applicationInfo;
    instanceCreateInfo.enabledExtensionCount =
        static_cast<uint32_t>(m_usedInstanceExtensions.size());
    instanceCreateInfo.ppEnabledExtensionNames = m_usedInstanceExtensions.data();
    instanceCreateInfo.enabledLayerCount       = static_cast<uint32_t>(m_usedInstanceLayers.size());
    instanceCreateInfo.ppEnabledLayerNames     = m_usedInstanceLayers.data();
    instanceCreateInfo.pNext                   = info.instanceCreateInfoExt;

    CHECK_EQ(vkCreateInstance(&instanceCreateInfo, nullptr, &m_instance), VK_SUCCESS);

    for (const auto& it : m_usedInstanceExtensions) {
        if (strcmp(it, VK_EXT_DEBUG_UTILS_EXTENSION_NAME) == 0) {
            initDebugUtils();
            break;
        }
    }

    return true;
}

//--------------------------------------------------------------------------------------------------
// Create Vulkan device
// \p deviceIndex is the index from the list of getPhysicalDevices/getPhysicalDeviceGroups
bool Context::InitDevice(uint32_t deviceIndex, const ContextCreateInfo& info)
{
    assert(m_instance != nullptr);

    VkPhysicalDeviceGroupProperties physicalGroup;
    if (info.useDeviceGroups) {
        auto groups = GetGpuGroups();
        assert(deviceIndex < static_cast<uint32_t>(groups.size()));
        physicalGroup    = groups[deviceIndex];
        m_physicalDevice = physicalGroup.physicalDevices[0];
    } else {
        auto physicalDevices = GetGpus();
        assert(deviceIndex < static_cast<uint32_t>(physicalDevices.size()));
        m_physicalDevice = physicalDevices[deviceIndex];
    }

    initPhysicalInfo(m_physicalInfo, m_physicalDevice, info.apiMajor, info.apiMinor);

    // features

    VkPhysicalDeviceFeatures2 features2{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2};
    Features11Old             features11old;

    features2.features = m_physicalInfo.features10;
    features11old.read(m_physicalInfo.features11);

    if (info.apiMajor == 1 && info.apiMinor == 1) {
        features2.pNext = &features11old.multiview;
    } else if (info.apiMajor == 1 && info.apiMinor >= 2) {
        features2.pNext                 = &m_physicalInfo.features11;
        m_physicalInfo.features11.pNext = &m_physicalInfo.features12;
        m_physicalInfo.features12.pNext = nullptr;
    }

    std::vector<VkDeviceQueueCreateInfo> queueCreateInfos;
    std::vector<float>                   priorities;
    std::vector<void*>                   featureStructs;

    bool queueFamilyGeneralPurpose = false;
    {
        for (auto& it : m_physicalInfo.queueProperties) {
            if ((it.queueFlags
                 & (VK_QUEUE_GRAPHICS_BIT | VK_QUEUE_COMPUTE_BIT | VK_QUEUE_TRANSFER_BIT))
                == (VK_QUEUE_GRAPHICS_BIT | VK_QUEUE_COMPUTE_BIT | VK_QUEUE_TRANSFER_BIT)) {
                queueFamilyGeneralPurpose = true;
            }
            if (it.queueCount > priorities.size()) {
                // set all priorities equal
                priorities.resize(it.queueCount, 1.0f);
            }
        }
        for (uint32_t i = 0; i < m_physicalInfo.queueProperties.size(); ++i) {
            VkDeviceQueueCreateInfo queueInfo{VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO};
            queueInfo.queueFamilyIndex = i;
            queueInfo.queueCount       = m_physicalInfo.queueProperties[i].queueCount;
            queueInfo.pQueuePriorities = priorities.data();

            queueCreateInfos.push_back(queueInfo);
        }
    }

    if (!queueFamilyGeneralPurpose) {
        LOG(WARNING) << ("could not find queue that supports graphics, compute and transfer");
    }

    // allow all queues
    VkDeviceCreateInfo deviceCreateInfo{VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO};
    deviceCreateInfo.queueCreateInfoCount = (uint32_t)queueCreateInfos.size();
    deviceCreateInfo.pQueueCreateInfos    = queueCreateInfos.data();

    // physical device extensions
    auto extensionProperties = GetGpuExtensions(m_physicalDevice);

    if (info.verboseAvailable) {
        LOG(INFO ) << ("_____________________________\n");
        LOG(INFO) << ("Available Device Extensions :\n");
        for (auto it : extensionProperties) {
            LOG(INFO) << ("%s (v. %d)\n", it.extensionName, it.specVersion);
        }
    }

    if (fillFilteredNameArray(m_usedDeviceExtensions, extensionProperties, info.deviceExtensions,
                              featureStructs)
        != VK_SUCCESS) {
        DeInit();
        return false;
    }

    if (info.verboseUsed) {
        LOG(INFO) << ("________________________\n");
        LOG(INFO) << ("Used Device Extensions :\n");
        for (auto it : m_usedDeviceExtensions) {
            LOG(INFO) << ("%s\n", it);
        }
        LOG(INFO) << ("\n");
    }

    struct ExtensionHeader  // Helper struct to link extensions together
    {
        VkStructureType sType;
        void*           pNext;
    };

    // use the features2 chain to append extensions
    if (!featureStructs.empty()) {
        // build up chain of all used extension features
        for (size_t i = 0; i < featureStructs.size(); i++) {
            auto* header  = reinterpret_cast<ExtensionHeader*>(featureStructs[i]);
            header->pNext = i < featureStructs.size() - 1 ? featureStructs[i + 1] : nullptr;
        }

        // append to the end of current feature2 struct
        ExtensionHeader* lastCoreFeature = (ExtensionHeader*)&features2;
        while (lastCoreFeature->pNext != nullptr) {
            lastCoreFeature = (ExtensionHeader*)lastCoreFeature->pNext;
        }
        lastCoreFeature->pNext = featureStructs[0];

        // query support
        vkGetPhysicalDeviceFeatures2(m_physicalDevice, &features2);
    }

    // disable some features
    if (info.disableRobustBufferAccess) {
        features2.features.robustBufferAccess = VK_FALSE;
    }

    deviceCreateInfo.enabledExtensionCount   = static_cast<uint32_t>(m_usedDeviceExtensions.size());
    deviceCreateInfo.ppEnabledExtensionNames = m_usedDeviceExtensions.data();

    // Vulkan >= 1.1 uses pNext to enable features, and not pEnabledFeatures
    deviceCreateInfo.pEnabledFeatures = nullptr;
    deviceCreateInfo.pNext            = &features2;

    // device group information
    VkDeviceGroupDeviceCreateInfo deviceGroupCreateInfo{
        VK_STRUCTURE_TYPE_DEVICE_GROUP_DEVICE_CREATE_INFO};
    if (info.useDeviceGroups) {
        // add ourselves to the chain
        deviceGroupCreateInfo.pNext               = deviceCreateInfo.pNext;
        deviceGroupCreateInfo.physicalDeviceCount = uint32_t(physicalGroup.physicalDeviceCount);
        deviceGroupCreateInfo.pPhysicalDevices    = physicalGroup.physicalDevices;
        deviceCreateInfo.pNext                    = &deviceGroupCreateInfo;
    }

    ExtensionHeader* deviceCreateChain = nullptr;
    if (info.deviceCreateInfoExt) {
        deviceCreateChain = (ExtensionHeader*)info.deviceCreateInfoExt;
        while (deviceCreateChain->pNext != nullptr) {
            deviceCreateChain = (ExtensionHeader*)deviceCreateChain->pNext;
        }
        // override last of external chain
        deviceCreateChain->pNext = (void*)deviceCreateInfo.pNext;
        deviceCreateInfo.pNext   = info.deviceCreateInfoExt;
    }

    VkResult result = vkCreateDevice(m_physicalDevice, &deviceCreateInfo, nullptr, &m_device);

    if (deviceCreateChain) {
        // reset last of external chain
        deviceCreateChain->pNext = nullptr;
    }

    if (result != VK_SUCCESS) {
        DeInit();
        return false;
    }

    load_VK_EXTENSION_SUBSET(m_instance, vkGetInstanceProcAddr, m_device, vkGetDeviceProcAddr);

    // Now pick 3 distinct queues for graphics, compute and transfer operations
    struct QueueScore {
        uint32_t score;  // the lower the score, the more 'specialized' it is
        uint32_t familyIndex;
        uint32_t queueIndex;
    };

    std::vector<QueueScore> queueScores;
    for (uint32_t qF = 0; qF < m_physicalInfo.queueProperties.size(); ++qF) {
        const auto& queueFamily = m_physicalInfo.queueProperties[qF];

        QueueScore score{0, qF, 0};
        if (queueFamily.queueFlags & VK_QUEUE_GRAPHICS_BIT) {
            score.score++;
        }
        if (queueFamily.queueFlags & VK_QUEUE_COMPUTE_BIT) {
            score.score++;
        }
        if (queueFamily.queueFlags & VK_QUEUE_TRANSFER_BIT) {
            score.score++;
        }
        for (uint32_t qI = 0; qI < queueFamily.queueCount; ++qI) {
            score.queueIndex = qI;
            queueScores.emplace_back(score);
        }
    }

    // Sort the queues for specialization, highest specialization has lowest score
    std::sort(queueScores.begin(), queueScores.end(),
              [](const QueueScore& lhs, const QueueScore& rhs) {
                  if (lhs.score < rhs.score)
                      return true;
                  if (lhs.score > rhs.score)
                      return false;
                  return lhs.queueIndex < rhs.queueIndex;
              });

    auto findQueue = [this, &queueScores](VkQueueFlags       needFlags,
                                                      const std::string& name) -> Queue {
        for (uint32_t q = 0; q < queueScores.size(); ++q) {
            const QueueScore& score  = queueScores[q];
            auto&             family = m_physicalInfo.queueProperties[score.familyIndex];
            if ((family.queueFlags & needFlags) == needFlags) {
                Queue queue;
                vkGetDeviceQueue(m_device, score.familyIndex, score.queueIndex, &queue.queue);
                queue.familyIndex = score.familyIndex;
                queue.queueIndex  = score.queueIndex;
                // we used this queue
                queueScores.erase(queueScores.begin() + q);
                return queue;
            }
        }
        return Queue();
    };

    m_queueGCT =
        findQueue(VK_QUEUE_GRAPHICS_BIT | VK_QUEUE_COMPUTE_BIT | VK_QUEUE_TRANSFER_BIT, "queueGCT");
    assert(m_queueGCT.familyIndex != ~uint32_t(0));

    m_queueC = findQueue(VK_QUEUE_COMPUTE_BIT, "queueC");
    m_queueT = findQueue(VK_QUEUE_TRANSFER_BIT, "queueT");

    return true;
}


//--------------------------------------------------------------------------------------------------
// Set the queue family index compatible with the \p surface
//
bool Context::SetGCTQueueWithPresent(VkSurfaceKHR surface)
{
    VkQueueFlags bits = VK_QUEUE_GRAPHICS_BIT | VK_QUEUE_COMPUTE_BIT | VK_QUEUE_TRANSFER_BIT;

    for (uint32_t i = 0; i < uint32_t(m_physicalInfo.queueProperties.size()); i++) {
        VkBool32 supportsPresent;
        vkGetPhysicalDeviceSurfaceSupportKHR(m_physicalDevice, i, surface, &supportsPresent);

        if ((supportsPresent == VK_TRUE)
            && ((m_physicalInfo.queueProperties[i].queueFlags & bits) == bits)) {
            vkGetDeviceQueue(m_device, i, 0, &m_queueGCT.queue);
            m_queueGCT.familyIndex = i;

            return true;
        }
    }

    return false;
}

// #UNSUED
uint32_t Context::getQueueFamily(VkQueueFlags flagsSupported, VkQueueFlags flagsDisabled,
                                 VkSurfaceKHR surface)
{
    uint32_t queueFamilyIndex = 0;
    for (auto& it : m_physicalInfo.queueProperties) {
        VkBool32 supportsPresent = VK_TRUE;
        if (surface) {
            supportsPresent = VK_FALSE;
            vkGetPhysicalDeviceSurfaceSupportKHR(m_physicalDevice, queueFamilyIndex, surface,
                                                 &supportsPresent);
        }

        if (supportsPresent && ((it.queueFlags & flagsSupported) == flagsSupported)
            && ((it.queueFlags & flagsDisabled) == 0)) {
            return queueFamilyIndex;
        }

        queueFamilyIndex++;
    }

    return ~0;
}

//--------------------------------------------------------------------------------------------------
// Destructor
//
void Context::DeInit()
{
    if (m_device) {
        VkResult result = vkDeviceWaitIdle(m_device);
        if (result != VK_SUCCESS) {
            exit(-1);
        }

        vkDestroyDevice(m_device, nullptr);
        m_device = VK_NULL_HANDLE;
    }
    if (m_destroyDebugUtilsMessengerEXT) {
        // Destroy the Debug Utils Messenger
        m_destroyDebugUtilsMessengerEXT(m_instance, m_dbgMessenger, nullptr);
    }

    if (m_instance) {
        vkDestroyInstance(m_instance, nullptr);
        m_instance = VK_NULL_HANDLE;
    }

    m_usedInstanceExtensions.clear();
    m_usedInstanceLayers.clear();
    m_usedDeviceExtensions.clear();

    m_createDebugUtilsMessengerEXT  = nullptr;
    m_destroyDebugUtilsMessengerEXT = nullptr;
    m_dbgMessenger                  = nullptr;

    reset_VK_EXTENSION_SUBSET();
}

bool Context::hasDeviceExtension(const char* name) const
{
    for (const auto& usedDeviceExtension : m_usedDeviceExtensions) {
        if (strcmp(name, usedDeviceExtension) == 0) {
            return true;
        }
    }
    return false;
}

bool Context::hasInstanceExtension(const char* name) const
{
    for (const auto& usedInstanceExtension : m_usedInstanceExtensions) {
        if (strcmp(name, usedInstanceExtension) == 0) {
            return true;
        }
    }
    return false;
}

//--------------------------------------------------------------------------------------------------
//
//
ContextCreateInfo::ContextCreateInfo(bool bUseValidation)
{
#ifdef _DEBUG
    instanceExtensions.push_back({VK_EXT_DEBUG_UTILS_EXTENSION_NAME, true});
    if (bUseValidation)
        instanceLayers.push_back({"VK_LAYER_KHRONOS_validation", true});
#endif
}

void ContextCreateInfo::AddInstanceExtension(const char* name, bool optional)
{
    instanceExtensions.emplace_back(name, optional);
}

void ContextCreateInfo::AddInstanceLayer(const char* name, bool optional)
{
    instanceLayers.emplace_back(name, optional);
}

//--------------------------------------------------------------------------------------------------
// pFeatureStruct must be provided if it exists, it will be queried from physical device
// and then passed in this state to device create info.
//
void ContextCreateInfo::AddDeviceExtension(const char* name, bool optional, void* pFeatureStruct,
                                           uint32_t version)
{
    deviceExtensions.emplace_back(name, optional, pFeatureStruct, version);
}

void ContextCreateInfo::removeInstanceExtension(const char* name)
{
    for (size_t i = 0; i < instanceExtensions.size(); i++) {
        if (strcmp(instanceExtensions[i].name, name) == 0) {
            instanceExtensions.erase(instanceExtensions.begin() + i);
        }
    }
}

void ContextCreateInfo::removeInstanceLayer(const char* name)
{
    for (size_t i = 0; i < instanceLayers.size(); i++) {
        if (strcmp(instanceLayers[i].name, name) == 0) {
            instanceLayers.erase(instanceLayers.begin() + i);
        }
    }
}

void ContextCreateInfo::removeDeviceExtension(const char* name)
{
    for (size_t i = 0; i < deviceExtensions.size(); i++) {
        if (strcmp(deviceExtensions[i].name, name) == 0) {
            deviceExtensions.erase(deviceExtensions.begin() + i);
        }
    }
}

void ContextCreateInfo::SetVersion(int major, int minor)
{
    assert(apiMajor >= 1 && apiMinor >= 1);
    apiMajor = major;
    apiMinor = minor;
}

VkResult Context::fillFilteredNameArray(Context::NameArray&                   used,
                                        const std::vector<VkLayerProperties>& properties,
                                        const ContextCreateInfo::EntryArray&  requested)
{
    for (auto itr = requested.begin(); itr != requested.end(); ++itr) {
        bool found = false;
        for (const auto& property : properties) {
            if (strcmp(itr->name, property.layerName) == 0) {
                found = true;
                break;
            }
        }

        if (!found && !itr->optional) {
            LOG(WARNING) << Format("VK_ERROR_EXTENSION_NOT_PRESENT: %s\n", itr->name);
            return VK_ERROR_EXTENSION_NOT_PRESENT;
        }

        used.push_back(itr->name);
    }

    return VK_SUCCESS;
}


VkResult Context::fillFilteredNameArray(Context::NameArray&                       used,
                                        const std::vector<VkExtensionProperties>& properties,
                                        const ContextCreateInfo::EntryArray&      requested,
                                        std::vector<void*>&                       featureStructs)
{
    for (const auto& itr : requested) {
        bool found = false;
        for (const auto& property : properties) {
            if (strcmp(itr.name, property.extensionName) == 0
                && (itr.version == 0 || itr.version == property.specVersion)) {
                found = true;
                break;
            }
        }

        if (found) {
            used.push_back(itr.name);
            if (itr.pFeatureStruct) {
                featureStructs.push_back(itr.pFeatureStruct);
            }
        } else if (!itr.optional) {
            LOG(WARNING) << Format("VK_ERROR_EXTENSION_NOT_PRESENT: %s - %d\n", itr.name, itr.version);
            return VK_ERROR_EXTENSION_NOT_PRESENT;
        }
    }

    return VK_SUCCESS;
}

void Context::initPhysicalInfo(PhysicalDeviceInfo& info, VkPhysicalDevice physicalDevice,
                               uint32_t versionMajor, uint32_t versionMinor)
{
    vkGetPhysicalDeviceMemoryProperties(physicalDevice, &info.memoryProperties);
    uint32_t count;
    vkGetPhysicalDeviceQueueFamilyProperties(physicalDevice, &count, nullptr);
    info.queueProperties.resize(count);
    vkGetPhysicalDeviceQueueFamilyProperties(physicalDevice, &count, info.queueProperties.data());

    // for queries and device creation
    VkPhysicalDeviceFeatures2   features2{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2};
    VkPhysicalDeviceProperties2 properties2 = {VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2};
    Properties11Old             properties11old;
    Features11Old               features11old;

    if (versionMajor == 1 && versionMinor == 1) {
        features2.pNext   = &features11old.multiview;
        properties2.pNext = &properties11old.maintenance3;
    } else if (versionMajor == 1 && versionMinor >= 2) {
        features2.pNext       = &info.features11;
        info.features11.pNext = &info.features12;
        info.features12.pNext = nullptr;

        info.properties12.driverID                     = VK_DRIVER_ID_NVIDIA_PROPRIETARY;
        info.properties12.supportedDepthResolveModes   = VK_RESOLVE_MODE_MAX_BIT;
        info.properties12.supportedStencilResolveModes = VK_RESOLVE_MODE_MAX_BIT;

        properties2.pNext       = &info.properties11;
        info.properties11.pNext = &info.properties12;
        info.properties12.pNext = nullptr;
    }

    vkGetPhysicalDeviceFeatures2(physicalDevice, &features2);
    vkGetPhysicalDeviceProperties2(physicalDevice, &properties2);

    info.properties10 = properties2.properties;
    info.features10   = features2.features;

    if (versionMajor == 1 && versionMinor == 1) {
        properties11old.write(info.properties11);
        features11old.write(info.features11);
    }
}

void Context::initDebugUtils()
{
    // Debug reporting system
    // Setup our pointers to the VK_EXT_debug_utils commands
    m_createDebugUtilsMessengerEXT =
        (PFN_vkCreateDebugUtilsMessengerEXT)vkGetInstanceProcAddr(m_instance,
                                                                  "vkCreateDebugUtilsMessengerEXT");
    m_destroyDebugUtilsMessengerEXT = (PFN_vkDestroyDebugUtilsMessengerEXT)vkGetInstanceProcAddr(
        m_instance, "vkDestroyDebugUtilsMessengerEXT");
    // Create a Debug Utils Messenger that will trigger our callback for any warning
    // or error.
    if (m_createDebugUtilsMessengerEXT != nullptr) {
        VkDebugUtilsMessengerCreateInfoEXT dbg_messenger_create_info;
        dbg_messenger_create_info.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT;
        dbg_messenger_create_info.pNext = nullptr;
        dbg_messenger_create_info.flags = 0;
        dbg_messenger_create_info.messageSeverity =
            VK_DEBUG_UTILS_MESSAGE_SEVERITY_INFO_BIT_EXT
            | VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT
            | VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;
        dbg_messenger_create_info.messageType = VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT
                                                | VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT
                                                | VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT;
        dbg_messenger_create_info.pfnUserCallback = debugMessengerCallback;
        dbg_messenger_create_info.pUserData       = this;
        VkResult result = m_createDebugUtilsMessengerEXT(m_instance, &dbg_messenger_create_info,
                                                         nullptr, &m_dbgMessenger);
    }
}

//--------------------------------------------------------------------------------------------------
// Returns the list of devices or groups compatible with the mandatory extensions
//
std::vector<int> Context::GetCompatibleDevices(const ContextCreateInfo& info)
{
    assert(m_instance != nullptr);
    std::vector<int>                             compatibleDevices;
    std::vector<VkPhysicalDeviceGroupProperties> groups;
    std::vector<VkPhysicalDevice>                physicalDevices;

    uint32_t nbElems;
    if (info.useDeviceGroups) {
        groups  = GetGpuGroups();
        nbElems = static_cast<uint32_t>(groups.size());
    } else {
        physicalDevices = GetGpus();
        nbElems         = static_cast<uint32_t>(physicalDevices.size());
    }

    if (info.verboseCompatibleDevices) {
        LOG(INFO) << ("____________________\n");
        LOG(INFO) << ("Compatible Devices :\n");
    }

    uint32_t compatible = 0;
    for (uint32_t elemId = 0; elemId < nbElems; elemId++) {
        VkPhysicalDevice physicalDevice =
            info.useDeviceGroups ? groups[elemId].physicalDevices[0] : physicalDevices[elemId];

        // Note: all physical devices in a group are identical
        if (HasMandatoryExtensions(physicalDevice, info, info.verboseCompatibleDevices)) {
            compatibleDevices.push_back(elemId);
            if (info.verboseCompatibleDevices) {
                VkPhysicalDeviceProperties props;
                vkGetPhysicalDeviceProperties(physicalDevice, &props);
                LOG(INFO) << ("%d: %s\n", compatible, props.deviceName);
                compatible++;
            }
        } else if (info.verboseCompatibleDevices) {
            VkPhysicalDeviceProperties props;
            vkGetPhysicalDeviceProperties(physicalDevice, &props);
            LOG(WARNING) << ("Skipping physical device %s\n", props.deviceName);
        }
    }
    if (info.verboseCompatibleDevices) {
        LOG(INFO) << Format("Physical devices found : %d", compatible);
        if (compatible > 0) {
            LOG(INFO) << Format("%d\n", compatible);
        } else {
            LOG(INFO) << ("OMG... NONE !!\n");
        }
    }

    return compatibleDevices;
}

#define NVVK_CHECK(x) CHECK_EQ(x, VK_SUCCESS)

//--------------------------------------------------------------------------------------------------
// Return true if all extensions in info, marked as required are available on the physicalDevice
//
bool Context::HasMandatoryExtensions(VkPhysicalDevice physicalDevice, const ContextCreateInfo& info,
                                     bool bVerbose)
{
    std::vector<VkExtensionProperties> extensionProperties;

    uint32_t count;
    NVVK_CHECK(vkEnumerateDeviceExtensionProperties(physicalDevice, nullptr, &count, nullptr));
    extensionProperties.resize(count);
    NVVK_CHECK(vkEnumerateDeviceExtensionProperties(physicalDevice, nullptr, &count,
                                                    extensionProperties.data()));
    extensionProperties.resize(std::min(extensionProperties.size(), size_t(count)));

    return checkEntryArray(extensionProperties, info.deviceExtensions, bVerbose);
}

bool Context::checkEntryArray(const std::vector<VkExtensionProperties>& properties,
                              const ContextCreateInfo::EntryArray& requested, bool bVerbose)
{
    for (const auto& itr : requested) {
        bool found = false;
        for (const auto& property : properties) {
            if (strcmp(itr.name, property.extensionName) == 0) {
                found = true;
                break;
            }
        }

        if (!found && !itr.optional) {
            if (bVerbose) {
                LOG(WARNING) << Format("Could NOT locate mandatory extension '%s'\n", itr.name);
            }
            return false;
        }
    }

    return true;
}

std::vector<VkPhysicalDevice> Context::GetGpus()
{
    uint32_t                      nbElems;
    std::vector<VkPhysicalDevice> physicalDevices;
    NVVK_CHECK(vkEnumeratePhysicalDevices(m_instance, &nbElems, nullptr));
    physicalDevices.resize(nbElems);
    NVVK_CHECK(vkEnumeratePhysicalDevices(m_instance, &nbElems, physicalDevices.data()));
    return physicalDevices;
}

std::vector<VkPhysicalDeviceGroupProperties> Context::GetGpuGroups()
{
    uint32_t                                     deviceGroupCount;
    std::vector<VkPhysicalDeviceGroupProperties> groups;
    NVVK_CHECK(vkEnumeratePhysicalDeviceGroups(m_instance, &deviceGroupCount, nullptr));
    groups.resize(deviceGroupCount);
    NVVK_CHECK(vkEnumeratePhysicalDeviceGroups(m_instance, &deviceGroupCount, groups.data()));
    return groups;
}

std::vector<VkLayerProperties> Context::GetInstanceLayers()
{
    uint32_t                       count;
    std::vector<VkLayerProperties> layerProperties;
    NVVK_CHECK(vkEnumerateInstanceLayerProperties(&count, nullptr));
    layerProperties.resize(count);
    NVVK_CHECK(vkEnumerateInstanceLayerProperties(&count, layerProperties.data()));
    layerProperties.resize(std::min(layerProperties.size(), size_t(count)));
    return layerProperties;
}

std::vector<VkExtensionProperties> Context::GetInstanceExtensions()
{
    uint32_t                           count;
    std::vector<VkExtensionProperties> extensionProperties;
    NVVK_CHECK(vkEnumerateInstanceExtensionProperties(nullptr, &count, nullptr));
    extensionProperties.resize(count);
    NVVK_CHECK(vkEnumerateInstanceExtensionProperties(nullptr, &count, extensionProperties.data()));
    extensionProperties.resize(std::min(extensionProperties.size(), size_t(count)));
    return extensionProperties;
}

std::vector<VkExtensionProperties> Context::GetGpuExtensions(VkPhysicalDevice physicalDevice)
{
    uint32_t                           count;
    std::vector<VkExtensionProperties> extensionProperties;
    NVVK_CHECK(vkEnumerateDeviceExtensionProperties(physicalDevice, nullptr, &count, nullptr));
    extensionProperties.resize(count);
    NVVK_CHECK(vkEnumerateDeviceExtensionProperties(physicalDevice, nullptr, &count,
                                                    extensionProperties.data()));
    extensionProperties.resize(std::min(extensionProperties.size(), size_t(count)));
    return extensionProperties;
}
#else

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

void ContextCreateInfo::AddDeviceExtension(const char* name, bool optional, void* p_feature_struct)
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
    vk::DynamicLoader         dl;
    PFN_vkGetInstanceProcAddr fp_vkGetInstanceProcAddr =
        dl.getProcAddress<PFN_vkGetInstanceProcAddr>("vkGetInstanceProcAddr");
    VULKAN_HPP_DEFAULT_DISPATCHER.init(fp_vkGetInstanceProcAddr);

    auto application_info =
        vk::ApplicationInfo()
            .setPApplicationName(context_ci.app_title)
            .setPEngineName(context_ci.app_engine_name)
            .setApiVersion(VK_MAKE_VERSION(context_ci.api_major, context_ci.api_minor, 0));
    if (context_ci.verbose_used) {
        u32 version = vk::enumerateInstanceVersion(static_loader_);
        LOG(INFO) << "Vulkan version: ";
        printf(" - available:  %d.%d.%d\n", VK_VERSION_MAJOR(version), VK_VERSION_MINOR(version),
               VK_VERSION_PATCH(version));
        printf(" - requesting: %d.%d.%d\n", context_ci.api_major, context_ci.api_minor, 0);
    }

    {  // Checks if all the required layers are supported.
        auto layer_properties = GetInstanceLayers();

        if (FillFilterNameArray(&used_instance_layers_, layer_properties,
                                context_ci.instance_layers)
            != vk::Result::eSuccess) {
            return false;
        }
        if (context_ci.verbose_available) {
            LOG(INFO) << "-------------------------------";
            LOG(INFO) << "Available Instance Layers: ";
            size_t max_width = 0;
            for (const auto& prop : layer_properties)
                max_width = std::max(max_width, strlen(prop.layerName));
            for (const auto& prop : layer_properties) {
                printf(" - %-*s (ver. %x %x) : %s\n", cast_u32(max_width + 1), prop.layerName,
                       prop.specVersion, prop.implementationVersion, prop.description);
            }
            putchar('\n');
        }
    }

    {  // Checks if all the required instance extensions are supported.
        auto extension_properties = GetInstanceExtensions();

        std::vector<void*> features_structs;
        if (FillFilterNameArray(&used_instance_extensions_, extension_properties,
                                context_ci.instance_extensions, &features_structs)
            != vk::Result::eSuccess)
            return false;
        if (context_ci.verbose_available) {
            size_t max_width = 0;
            for (const auto& prop : extension_properties)
                max_width = std::max(max_width, strlen(prop.extensionName));
            LOG(INFO) << "\nAvailable Instance Extensions :";
            for (auto prop : extension_properties) {
                printf(" - %-*s (ver. %d)\n", cast_i32(max_width + 1), prop.extensionName,
                       prop.specVersion);
            }
            putchar('\n');
        }
    }
    if (context_ci.verbose_used) {
        LOG(INFO) << "-----------------\nUsed Instance Layers: ";
        for (auto layer : used_instance_layers_) {
            printf(" - %s\n", layer);
        }
        putchar('\n');
        LOG(INFO) << "-----------------\nUsed Instance Extensions: ";
        for (auto ext : used_instance_extensions_) {
            printf(" - %s\n", ext);
        }
        putchar('\n');
    }

    auto instance_ci = vk::InstanceCreateInfo()
                           .setPApplicationInfo(&application_info)
                           .setPEnabledExtensionNames(used_instance_extensions_)
                           .setPEnabledLayerNames(used_instance_layers_);
    instance_ci.setPNext(context_ci.instance_create_info_ext);

    instance_ = vk::createInstance(instance_ci, nullptr, static_loader_);
    VULKAN_HPP_DEFAULT_DISPATCHER.init(instance_);

    // Creates the debug util messenger if the corresponding extension exists.
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
        gpu_info_.features_120.pNext = nullptr;
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

    // Prepares the information to create the device.
    // --------------------------------------------------------------------------------------------
    auto device_ci = vk::DeviceCreateInfo().setQueueCreateInfos(queue_cis);

    auto extension_props = GetGpuExtensions(gpu_);

    if (context_ci.verbose_available) {
        LOG(INFO) << "\nAvailable GPU Extensions: ";
        size_t max_width = 0;
        for (auto& prop : extension_props)
            max_width = std::max(max_width, strlen(prop.extensionName));
        for (auto& prop : extension_props) {
            printf(" - %-*s (ver. %d)\n", cast_u32(max_width + 1), prop.extensionName,
                   prop.specVersion);
        }
        putchar('\n');
    }

    if (FillFilterNameArray(&used_device_extensions_, extension_props, context_ci.device_extensions,
                            &feature_structures)
        != vk::Result::eSuccess) {
        DeInit();
        return false;
    }

    if (context_ci.verbose_used) {
        LOG(INFO) << "\nUsed GPU Extensions :";
        for (auto& ext : used_device_extensions_) {
            printf(" - %s\n", ext);
        }
        putchar('\n');
    }

    struct ExtensionHeader {
        vk::StructureType struct_type;
        void*             p_next;
    };

    // Organizes all feature structures as a chain and appends it at the end of features2 chain.
    if (!feature_structures.empty()) {
        feature_structures.push_back(nullptr);
        for (size_t i = 0; i < feature_structures.size()-1; ++i) {
            auto* header = reinterpret_cast<ExtensionHeader*>(feature_structures[i]);
            CHECK(header);
            header->p_next = feature_structures[i + 1];
        }

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
        device_group_ci.pNext = device_ci.pNext;
        device_ci.pNext       = &device_group_ci;
        device_group_ci.setPhysicalDevices(gpu_group_properties.physicalDevices);
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

    // Loads extensions (function pointers) supported by the device.
    // --------------------------------------------------------------------------------------------
    load_VK_EXTENSION_SUBSET(instance_, vkGetInstanceProcAddr, device_, vkGetDeviceProcAddr);

    VULKAN_HPP_DEFAULT_DISPATCHER.init(device_);

    // Retrieves default queues.
    // --------------------------------------------------------------------------------------------
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

    CHECK_NE(queue_graphics_compute_transfer_.family_index, ~(0u));
    CHECK_NE(queue_transfer_.family_index, ~(0u));
    CHECK_NE(queue_compute_.family_index, ~(0u));
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
    for (const auto& entry : requested) {
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
                                        std::vector<void*>* feature_structs)
{
    CHECK(feature_structs);
    CHECK(used);
    for (const auto& entry : requested) {
        // Looks for the extension name in the supported properties.
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
                feature_structs->push_back(entry.p_feature_struct);
        } else if (!entry.optional) {
            LOG(WARNING) << "vk::Result::eErrorExtensionNotPresent: " << entry.name << " - "
                         << entry.version;
            return vk::Result::eErrorExtensionNotPresent;
        }
    }
    return vk::Result::eSuccess;
}

bool Context::CheckEntryArray(const std::vector<vk::ExtensionProperties>&  supported_properties,
                              const std::vector<ContextCreateInfo::Entry>& requested,
                              bool                                         verbose) const
{
    std::set<std::string> supported_names;
    for (const auto& prop : supported_properties)
        supported_names.insert(prop.extensionName);
    for (const auto& entry : requested) {
        // Checks if `entry` is in properties.
        auto iter = supported_names.find(entry.name);
        if (iter == supported_names.end() && !entry.optional) {
            LOG_IF(WARNING, verbose) << "Can't locate mandatory extension " << entry.name;
            return false;
        }
    }
    return true;
}

// static method
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
#endif // end of USE_NV_CONTEXT

}  // namespace vkpbr
