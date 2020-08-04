#include "vk_swapchain.h"

#include "logging.h"

namespace vkpbr {


bool SwapChain::Init(vk::Device         device,
                     vk::PhysicalDevice gpu,
                     vk::Queue          queue,
                     u32                queue_family_index,
                     vk::SurfaceKHR     surface,
                     vk::Format         format)
{
    CHECK(!device_);
    device_                 = device;
    gpu_                    = gpu;
    queue_                  = queue;
    queue_family_index_     = queue_family_index;
    surface_                = surface;
    change_id_              = 0;
    current_semaphore_index_ = 0;

    // Retrieves the list of formats that are supported.
    std::vector<vk::SurfaceFormatKHR> supported_formats = gpu_.getSurfaceFormatsKHR(surface_);
    CHECK(!supported_formats.empty());

    // Sets up the fallback option of surface format and colorspace.
    surface_format_ = vk::Format::eB8G8R8A8Unorm;
    surface_color_  = supported_formats.front().colorSpace;

    for (const auto& candidate_format : supported_formats) {
        if (candidate_format.format == format) {
            surface_format_ = format;
            surface_color_  = candidate_format.colorSpace;
            return true;
        }
    }

    LOG_ERROR << "no supported formats";
    return false;
}

void SwapChain::Deinit()
{
    this->DeinitResources();

    gpu_       = nullptr;
    device_    = nullptr;
    surface_   = nullptr;
    change_id_ = 0;
}

void SwapChain::Update(int width, int height)
{
    change_id_++;

    vk::SwapchainKHR old_swap_chain = swapchain_;
    device_.waitIdle();

    // Checks the surface capabilities and formats.
    auto                            surface_capabilities = gpu_.getSurfaceCapabilitiesKHR(surface_);
    std::vector<vk::PresentModeKHR> present_modes        = gpu_.getSurfacePresentModesKHR(surface_);

    vk::Extent2D swap_chain_extent;
    if (surface_capabilities.currentExtent.width == cast_u32(-1)) {
        CHECK_EQ(surface_capabilities.currentExtent.height, cast_u32(-1));
        // If the surface size is undefined, the size is set to requested size.
        swap_chain_extent.width  = width;
        swap_chain_extent.height = height;
    } else {
        // If the surface size is defined, then swap chain size must match.
        swap_chain_extent = surface_capabilities.currentExtent;
    }
    CHECK_GT(swap_chain_extent.width, 0);
    CHECK_GT(swap_chain_extent.height, 0);

    vk::PresentModeKHR swap_chain_present_mode = vk::PresentModeKHR::eFifo;
    for (auto candidate_mode : present_modes) {
        if (candidate_mode == vk::PresentModeKHR::eMailbox) {
            swap_chain_present_mode = candidate_mode;
            break;
        } else if (candidate_mode == vk::PresentModeKHR::eImmediate) {
            swap_chain_present_mode = candidate_mode;
        }
    }

    // Determines the number of vk::Image's to use in the swap chain.
    // 1 image at a time is desired besides the images being displayed and queued for display.
    u32 desired_num_swap_chain_images = surface_capabilities.minImageCount + 1;
    if ((surface_capabilities.maxImageCount > 0)
        && (desired_num_swap_chain_images > surface_capabilities.maxImageCount)) {
        // Application must settle for fewer images than desired.
        desired_num_swap_chain_images = surface_capabilities.maxImageCount;
    }

    vk::SurfaceTransformFlagBitsKHR pre_transform;
    if (surface_capabilities.supportedTransforms & vk::SurfaceTransformFlagBitsKHR::eIdentity)
        pre_transform = vk::SurfaceTransformFlagBitsKHR::eIdentity;
    else
        pre_transform = surface_capabilities.currentTransform;

    // (Re-)creates the swap chain.
    auto swap_chain_ci = vk::SwapchainCreateInfoKHR()
                             .setSurface(surface_)
                             .setMinImageCount(desired_num_swap_chain_images)
                             .setImageFormat(surface_format_)
                             .setImageColorSpace(surface_color_)
                             .setImageExtent(swap_chain_extent)
                             .setImageUsage(vk::ImageUsageFlagBits::eColorAttachment
                                            | vk::ImageUsageFlagBits::eStorage
                                            | vk::ImageUsageFlagBits::eTransferDst)
                             .setPreTransform(pre_transform)
                             .setCompositeAlpha(vk::CompositeAlphaFlagBitsKHR::eOpaque)
                             .setImageArrayLayers(1)
                             .setImageSharingMode(vk::SharingMode::eExclusive)
                             .setQueueFamilyIndexCount(1)
                             .setPQueueFamilyIndices(&queue_family_index_)
                             .setPresentMode(swap_chain_present_mode)
                             .setOldSwapchain(old_swap_chain)
                             .setClipped(VK_TRUE);
    try {
        swapchain_ = device_.createSwapchainKHR(swap_chain_ci);
    } catch (std::exception& e) {
        LOG(FATAL) << "error creating the swap chain: " << e.what();
    }

    // If the swap chain was re-created, destroys the old one.
    if (!old_swap_chain) {
        for (auto& entry : entries_) {
            device_.destroyImageView(entry.image_view);
            device_.destroySemaphore(entry.read_semaphore);
            device_.destroySemaphore(entry.written_semaphore);
        }
        device_.destroySwapchainKHR(old_swap_chain);
    }

    auto swap_chain_images = device_.getSwapchainImagesKHR(swapchain_);
    entries_.resize(swap_chain_images.size());
    barriers_.resize(swap_chain_images.size());
    image_count_ = swap_chain_images.size();

    const auto kDefaultComponentMapping = vk::ComponentMapping()
                                              .setA(vk::ComponentSwizzle::eA)
                                              .setB(vk::ComponentSwizzle::eB)
                                              .setG(vk::ComponentSwizzle::eG)
                                              .setR(vk::ComponentSwizzle::eR);

    for (u32 i = 0; i < swap_chain_images.size(); ++i) {
        entries_[i].image = swap_chain_images[i];
        auto image_view_ci =
            vk::ImageViewCreateInfo()
                .setViewType(vk::ImageViewType::e2D)
                .setImage(entries_[i].image)
                .setFormat(surface_format_)
                .setComponents(kDefaultComponentMapping)
                .setSubresourceRange({vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1});
        entries_[i].image_view = device_.createImageView(image_view_ci);

        auto sem_ci                   = vk::SemaphoreCreateInfo();
        entries_[i].read_semaphore    = device_.createSemaphore(sem_ci);
        entries_[i].written_semaphore = device_.createSemaphore(sem_ci);

        // Initial barriers.
        auto range = vk::ImageSubresourceRange()
                         .setAspectMask(vk::ImageAspectFlagBits::eColor)
                         .setBaseMipLevel(0)
                         .setLevelCount(VK_REMAINING_MIP_LEVELS)
                         .setBaseArrayLayer(0)
                         .setLayerCount(VK_REMAINING_ARRAY_LAYERS);
        barriers_[i] = vk::ImageMemoryBarrier()
                           .setDstAccessMask({})
                           .setSrcAccessMask({})
                           .setOldLayout(vk::ImageLayout::eUndefined)
                           .setNewLayout(vk::ImageLayout::ePresentSrcKHR)
                           .setImage(entries_[i].image)
                           .setSubresourceRange(range);
    }
    CHECK_GT(image_count_, 0);

    width_ = width;
    height_ = height;

    current_semaphore_index_ = 0;
    current_image_index_ = 0;
}

bool SwapChain::Acquire()
{
    return AcquireCustom(ActiveReadSemaphore());
}

bool SwapChain::AcquireCustom(vk::Semaphore semaphore)
{
    // Tries re-creation a few times.
    for (int i = 0; i < 2; ++i) {
        auto image_index = device_.acquireNextImageKHR(swapchain_, UINT64_MAX, semaphore, nullptr);
        switch (image_index.result) {
            case vk::Result::eSuccess:
                current_image_index_ = image_index.value;
                return true;
            case vk::Result::eErrorOutOfDateKHR:
            case vk::Result::eSuboptimalKHR:
                device_.waitIdle();
                DeinitResources();
                Update(width_, height_);
                break;
            default:
                return false;
        }
    }
    return false;
}

vk::Image SwapChain::GetImage(u32 i) const
{
    if (i > image_count_) {
        LOG_ERROR << "image index out of bound";
        return nullptr;
    }
    return entries_[i].image;
}

vk::ImageView SwapChain::GetImageView(u32 i) const {
    if (i > image_count_) {
        LOG_ERROR << "image view index out of bound";
        return nullptr;
    }
    return entries_[i].image_view;
}

void SwapChain::Present(vk::Queue queue) {
    auto present_info = vk::PresentInfoKHR();

    PreparePresentInfo(&present_info);
    vk::Result present_result = queue.presentKHR(present_info);
    CHECK_EQ(present_result, vk::Result::eSuccess);
}

void SwapChain::PreparePresentInfo(vk::PresentInfoKHR * present_info) {
    vk::Semaphore& written_semaphore =
        entries_[current_semaphore_index_ % image_count_].written_semaphore;

    (*present_info).setSwapchainCount(1)
        .setWaitSemaphoreCount(1)
        .setPWaitSemaphores(&written_semaphore)
        .setPSwapchains(&swapchain_)
        .setPImageIndices(&current_image_index_);

    current_semaphore_index_++;
}

void SwapChain::DeinitResources() {
    if (!device_)
        return;
    device_.waitIdle();

    for (auto entry : entries_) {
        device_.destroyImageView(entry.image_view);
        device_.destroySemaphore(entry.read_semaphore);
        device_.destroySemaphore(entry.written_semaphore);
    }

    if (swapchain_) {
        device_.destroySwapchainKHR(swapchain_);
        swapchain_ = nullptr;
    }

    entries_.clear();
    barriers_.clear();
}

void SwapChain::CmdUpdateBarriers(vk::CommandBuffer cmd) const
{
    cmd.pipelineBarrier(vk::PipelineStageFlagBits::eTopOfPipe,
                        vk::PipelineStageFlagBits::eTopOfPipe, {}, 0, nullptr, 0, nullptr,
                        image_count_, barriers_.data());
}

}  // namespace vkpbr