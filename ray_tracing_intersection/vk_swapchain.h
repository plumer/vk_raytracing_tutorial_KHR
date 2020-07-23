#ifndef NVCOPY_VK_SWAPCHAIN_H_
#define NVCOPY_VK_SWAPCHAIN_H_

#include "types.h"
#include <vector>
#include <vulkan/vulkan.hpp>

namespace vkpbr {

/** 
 * Class SwapChain
 * 
 * For each swapchain image there is an image view, and one read+write semaphore. Furthermore,
 * there is a utility function to setup the image transitions from vk::ImageLayout::eUndefined to
 * vk::ImageLayout::ePresentSrcKHR.
 */
class SwapChain
{
  public:
    SwapChain() = default;
    ~SwapChain() {}

    SwapChain(const SwapChain&) = delete;
    SwapChain& operator=(const SwapChain&) = delete;

    bool Init(vk::Device         device,
              vk::PhysicalDevice gpu,
              vk::Queue          queue,
              u32                queue_family_index,
              vk::SurfaceKHR     surface,
              vk::Format         format = vk::Format::eB8G8R8A8Unorm);
    void Deinit();

    // Updates the swapchain configuration.
    void Update(int width, int height);

    // Sets active index.
    bool Acquire();
    bool AcquireCustom(vk::Semaphore semaphore);

    vk::Semaphore ActiveReadSemaphore() const
    {
        return entries_[(current_semaphore_index_ % image_count_)].read_semaphore;
    }

    vk::Semaphore ActiveWrittenSemaphore() const
    {
        return entries_[(current_semaphore_index_ % image_count_)].written_semaphore;
    }

    vk::Image     ActiveImage() const { return entries_[current_image_index_].image; }
    vk::ImageView ActiveImageView() const { return entries_[current_image_index_].image_view; }
    u32           ActiveImageIndex() const { return current_image_index_; }

    vk::SwapchainKHR Swapchain() const { return swapchain_; }
    u32              NumImages() const { return image_count_; }
    vk::Image        GetImage(u32 i) const;
    vk::ImageView    GetImageView(u32 i) const;
    vk::Format       Format() const { return surface_format_; }
    u32              Width() const { return width_; }
    u32              Height() const { return height_; }
    u32              ChangeID() const { return change_id_; }

    void Present() { Present(queue_); }
    void Present(vk::Queue);
    void PreparePresentInfo(vk::PresentInfoKHR * present_info);

    void CmdUpdateBarriers(vk::CommandBuffer cmd) const;

  private:
    struct Entry
    {
        vk::Image     image;
        vk::ImageView image_view;
        vk::Semaphore read_semaphore;
        vk::Semaphore written_semaphore;
    };

    // Handles to context objects.
    // ------------------------------------------------------------------------
    vk::Device         device_;
    vk::PhysicalDevice gpu_;

    vk::Queue queue_;
    u32       queue_family_index_ = 0;

    vk::SurfaceKHR    surface_;
    vk::Format        surface_format_;
    vk::ColorSpaceKHR surface_color_;

    u32              image_count_ = 0;
    vk::SwapchainKHR swapchain_;

    std::vector<Entry>                  entries_;
    std::vector<vk::ImageMemoryBarrier> barriers_;

    // Index for current image, acquired from vkAcquireNextImageKHR.
    // Vulkan spec:
    // > The order in which images are acquired is implementation-dependent, and maybe different
    // > from the order the images were presented.
    u32 current_image_index_     = 0;
    u32 current_semaphore_index_ = 0;
    u32 change_id_               = 0;
    // Surface width and height.
    u32 width_  = 0;
    u32 height_ = 0;

    void DeinitResources();
};

}  // namespace vkpbr

#endif  // !NVCOPY_VK_SWAPCHAIN_H_
