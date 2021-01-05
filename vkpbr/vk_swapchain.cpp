/* Copyright (c) 2014-2018, NVIDIA CORPORATION. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of NVIDIA CORPORATION nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "vk_swapchain.h"

#include <assert.h>
#include "logging.h"

namespace vkpbr {
bool SwapChain::Init(vk::Device device, vk::PhysicalDevice gpu, vk::Queue queue,
                     uint32_t queueFamilyIndex, vk::SurfaceKHR surface, vk::Format format)
{
    assert(!m_device);
    m_device           = device;
    m_physicalDevice   = gpu;
    m_swapchain        = nullptr;
    m_queue            = queue;
    m_queueFamilyIndex = queueFamilyIndex;
    m_changeID         = 0;
    m_currentSemaphore = 0;
    m_surface          = surface;


    // Get the list of vk::Format's that are supported:

    auto surface_formats = m_physicalDevice.getSurfaceFormatsKHR(m_surface);
    assert(!surface_formats.empty());
    // If the format list includes just one entry of VK_FORMAT_UNDEFINED,
    // the surface has no preferred format.  Otherwise, at least one
    // supported format will be returned.

    m_surfaceFormat = vk::Format::eB8G8R8A8Unorm;
    m_surfaceColor  = surface_formats[0].colorSpace;

    for (uint32_t i = 0; i < surface_formats.size(); i++) {
        if (surface_formats[i].format == vk::Format(format)) {
            m_surfaceFormat = surface_formats[i].format;
            m_surfaceColor  = surface_formats[i].colorSpace;
            return true;
        }
    }

    return false;
}

vk::Extent2D SwapChain::update(int width, int height, bool vsync)
{
    m_changeID++;

    vk::SwapchainKHR oldSwapchain = m_swapchain;

    m_device.waitIdle();

    auto surf_capabilities = m_physicalDevice.getSurfaceCapabilitiesKHR(m_surface);
    auto present_modes     = m_physicalDevice.getSurfacePresentModesKHR(m_surface);


    vk::Extent2D swapchainExtent;
    // width and height are either both -1, or both not -1.
    if (surf_capabilities.currentExtent.width == (uint32_t)-1) {
        // If the surface size is undefined, the size is set to
        // the size of the images requested.
        swapchainExtent.width  = width;
        swapchainExtent.height = height;
    } else {
        // If the surface size is defined, the swap chain size must match
        swapchainExtent = surf_capabilities.currentExtent;
    }

    // test against valid size, typically hit when windows are minimized, the app must
    // prevent triggering this code accordingly
    assert(swapchainExtent.width && swapchainExtent.height);

    // everyone must support FIFO mode
    vk::PresentModeKHR swapchainPresentMode = vk::PresentModeKHR::eFifo;
    // no vsync try to find a faster alternative to FIFO
    if (!vsync) {
        for (uint32_t i = 0; i < present_modes.size(); i++) {
            if (present_modes[i] == vk::PresentModeKHR::eMailbox) {
                // prefer mailbox due to no tearing
                swapchainPresentMode = vk::PresentModeKHR::eMailbox;
                break;
            }
            if (present_modes[i] == vk::PresentModeKHR::eImmediate) {
                swapchainPresentMode = vk::PresentModeKHR::eImmediate;
            }
        }
    }

    // Determine the number of VkImage's to use in the swap chain (we desire to
    // own only 1 image at a time, besides the images being displayed and
    // queued for display):
    uint32_t desiredNumberOfSwapchainImages = surf_capabilities.minImageCount + 1;
    if ((surf_capabilities.maxImageCount > 0)
        && (desiredNumberOfSwapchainImages > surf_capabilities.maxImageCount)) {
        // Application must settle for fewer images than desired:
        desiredNumberOfSwapchainImages = surf_capabilities.maxImageCount;
    }

    vk::SurfaceTransformFlagBitsKHR preTransform;
    if (surf_capabilities.supportedTransforms & vk::SurfaceTransformFlagBitsKHR::eIdentity) {
        preTransform = vk::SurfaceTransformFlagBitsKHR::eIdentity;
    } else {
        preTransform = surf_capabilities.currentTransform;
    }

    using vkIU = vk::ImageUsageFlagBits;

    vk::SwapchainCreateInfoKHR swapchain_ci;
    swapchain_ci.surface                  = m_surface;
    swapchain_ci.minImageCount            = desiredNumberOfSwapchainImages;
    swapchain_ci.imageFormat              = m_surfaceFormat;
    swapchain_ci.imageColorSpace          = m_surfaceColor;
    swapchain_ci.imageExtent              = swapchainExtent;
    swapchain_ci.imageUsage = vkIU::eColorAttachment | vkIU::eStorage | vkIU::eTransferDst;
    swapchain_ci.preTransform          = preTransform;
    swapchain_ci.compositeAlpha        = vk::CompositeAlphaFlagBitsKHR::eOpaque;
    swapchain_ci.imageArrayLayers      = 1;
    swapchain_ci.imageSharingMode      = vk::SharingMode::eExclusive;
    swapchain_ci.queueFamilyIndexCount = 1;
    swapchain_ci.pQueueFamilyIndices   = &m_queueFamilyIndex;
    swapchain_ci.presentMode           = swapchainPresentMode;
    swapchain_ci.oldSwapchain          = oldSwapchain;
    swapchain_ci.clipped               = true;

    //err = vkCreateSwapchainKHR(m_device, &swapchain_ci, nullptr, &m_swapchain);
    m_swapchain = m_device.createSwapchainKHR(swapchain_ci);
    CHECK(m_swapchain);

    // If we just re-created an existing swapchain, we should destroy the old
    // swapchain at this point.
    // Note: destroying the swapchain also cleans up all its associated
    // presentable images once the platform is done with them.
    if (oldSwapchain) {
        for (auto it : m_entries) {
            m_device.destroyImageView(it.imageView);
            m_device.destroySemaphore(it.readSemaphore);
            m_device.destroySemaphore(it.writtenSemaphore);
        }
        m_device.destroySwapchainKHR(oldSwapchain);
    }

    auto images = m_device.getSwapchainImagesKHR(m_swapchain);
    m_imageCount = images.size();

    m_entries.resize(m_imageCount);
    m_barriers.resize(m_imageCount);

    //
    // Image views
    //
    for (uint32_t i = 0; i < m_imageCount; i++) {
        Entry& entry = m_entries[i];

        // image
        entry.image = images[i];

        using vkCS = vk::ComponentSwizzle;
        auto component_mapping = vk::ComponentMapping(vkCS::eR, vkCS::eG, vkCS::eB, vkCS::eA);

        auto image_view_ci = vk::ImageViewCreateInfo()
                                 .setImage(entry.image)
                                 .setViewType(vk::ImageViewType::e2D)
                                 .setFormat(m_surfaceFormat)
                                 .setComponents(component_mapping);
        image_view_ci.subresourceRange = {vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1};

        entry.imageView = m_device.createImageView(image_view_ci);

        // semaphore
        vk::SemaphoreCreateInfo semCreateInfo = {};

        entry.readSemaphore = m_device.createSemaphore(semCreateInfo);
        entry.writtenSemaphore = m_device.createSemaphore(semCreateInfo);

        // initial barriers
        VkImageSubresourceRange range = {0};
        range.aspectMask              = VK_IMAGE_ASPECT_COLOR_BIT;
        range.baseMipLevel            = 0;
        range.levelCount              = VK_REMAINING_MIP_LEVELS;
        range.baseArrayLayer          = 0;
        range.layerCount              = VK_REMAINING_ARRAY_LAYERS;

        VkImageMemoryBarrier memBarrier = {VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER};
        memBarrier.sType                = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
        memBarrier.dstAccessMask        = 0;
        memBarrier.srcAccessMask        = 0;
        memBarrier.oldLayout            = VK_IMAGE_LAYOUT_UNDEFINED;
        memBarrier.newLayout            = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
        memBarrier.image                = entry.image;
        memBarrier.subresourceRange     = range;

        m_barriers[i] = memBarrier;
    }


    m_updateWidth  = width;
    m_updateHeight = height;
    m_vsync        = vsync;
    m_extent       = swapchainExtent;

    m_currentSemaphore = 0;
    m_currentImage     = 0;

    return swapchainExtent;
}

void SwapChain::deinitResources()
{
    if (!m_device)
        return;

    m_device.waitIdle();

    for (auto it : m_entries) {
        m_device.destroyImageView(it.imageView);
        m_device.destroySemaphore(it.readSemaphore);
        m_device.destroySemaphore(it.writtenSemaphore);
    }

    if (m_swapchain) {
        m_device.destroySwapchainKHR(m_swapchain);
        m_swapchain = nullptr;
    }

    m_entries.clear();
    m_barriers.clear();
}

void SwapChain::DeInit()
{
    deinitResources();

    m_physicalDevice = nullptr;
    m_device         = nullptr;
    m_surface        = nullptr;
    m_changeID       = 0;
}

bool SwapChain::acquire()
{
    return acquireCustom(getActiveReadSemaphore());
}

bool SwapChain::acquireCustom(vk::Semaphore semaphore)
{
    // try recreation a few times
    for (int i = 0; i < 2; i++) {
        //result = vkAcquireNextImageKHR(m_device, m_swapchain, UINT64_MAX, semaphore,
                                       //(vk::Fence)VK_NULL_HANDLE, &m_currentImage);
        auto [result, currentImage] =
            m_device.acquireNextImageKHR(m_swapchain, UINT64_MAX, semaphore, nullptr);
        if (result == vk::Result::eSuccess) {
            m_currentImage = currentImage;
            return true;
        } else if (result == vk::Result::eErrorOutOfDateKHR
                   || result == vk::Result::eSuboptimalKHR) {
            m_currentImage = currentImage;
            deinitResources();
            update(m_updateWidth, m_updateHeight, m_vsync);
        } else {
            return false;
        }
    }

    return false;
}

vk::Semaphore SwapChain::getActiveWrittenSemaphore() const
{
    return m_entries[(m_currentSemaphore % m_imageCount)].writtenSemaphore;
}

vk::Semaphore SwapChain::getActiveReadSemaphore() const
{
    return m_entries[(m_currentSemaphore % m_imageCount)].readSemaphore;
}

vk::Image SwapChain::getActiveImage() const
{
    return m_entries[m_currentImage].image;
}

vk::ImageView SwapChain::getActiveImageView() const
{
    return m_entries[m_currentImage].imageView;
}

vk::Image SwapChain::getImage(uint32_t i) const
{
    if (i >= m_imageCount)
        return nullptr;
    return m_entries[i].image;
}

void SwapChain::present(vk::Queue queue)
{
    vk::PresentInfoKHR presentInfo;

    presentCustom(&presentInfo);

    vk::Result present_result = queue.presentKHR(presentInfo);
    if (present_result != vk::Result::eSuccess) {
        LOG(ERROR) << "Unsuccessful result: " << vk::to_string(present_result);
    }
    // assert(result == VK_SUCCESS); // can fail on application exit
}

void SwapChain::presentCustom(vk::PresentInfoKHR * presentInfo)
{
    vk::Semaphore& written = m_entries[(m_currentSemaphore % m_imageCount)].writtenSemaphore;

    presentInfo->setSwapchains(m_swapchain).setWaitSemaphores(written);
    presentInfo->setImageIndices(m_currentImage);

    m_currentSemaphore++;
}

void SwapChain::cmdUpdateBarriers(vk::CommandBuffer cmd) const
{
    CHECK_EQ(m_barriers.size(), m_imageCount);
    cmd.pipelineBarrier(vk::PipelineStageFlagBits::eTopOfPipe,
                        vk::PipelineStageFlagBits::eTopOfPipe, {}, {}, {}, m_barriers);
}

uint32_t SwapChain::getChangeID() const
{
    return m_changeID;
}

vk::ImageView SwapChain::getImageView(uint32_t i) const
{
    if (i >= m_imageCount)
        return nullptr;
    return m_entries[i].imageView;
}

}  // namespace nvvk
