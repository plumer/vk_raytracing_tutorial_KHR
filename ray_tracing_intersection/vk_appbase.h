#ifndef NVCOPY_VK_APPBASE_H_
#define NVCOPY_VK_APPBASE_H_

#include <vector>
#include <vulkan/vulkan.hpp>

#ifdef _WIN32
#define GLFW_EXPOSE_NATIVE_WIN32
#endif  // _WIN32
#include <GLFW/glfw3.h>
#include <GLFW/glfw3native.h>

#include <glm/glm.hpp>

#include "camera.h"
#include "types.h"
#include "vk_swapchain.h"

namespace vkpbr {

class AppBase
{
  public:
    AppBase()          = default;
    virtual ~AppBase() = default;

    void SetCameraNavigator(std::shared_ptr<CameraNavigator>& navigator) {
        camera_navigator_ = navigator;
    }

    virtual void ResizeCallback(int width, int height) {}

    // Sets up the Application with Vulkan context objects.
    virtual void Setup(const vk::Instance& instance, const vk::Device& device,
                       const vk::PhysicalDevice& gpu, u32 graphics_queue_index);

    virtual void UnloadContext();

    vk::SurfaceKHR AcquireSurface(const vk::Instance& instance, GLFWwindow* window);

    void MakeSurface(const vk::SurfaceKHR& surface, u32 width, u32 height,
                     vk::Format color_format = vk::Format::eB8G8R8A8Unorm,
                     vk::Format depth_format = vk::Format::eD32SfloatS8Uint);

    // Creates framebuffers in which the image will be rendered.
    virtual void MakeFrameBuffers();

    virtual void MakeRenderPass();

    virtual void MakeDepthBuffer();

    void PrepareFrame();

    void SubmitFrame();

    void SetViewport(const vk::CommandBuffer& cmd_buffer);

    virtual void WindowResizeCallback(int width, int height);
    virtual void MouseMotionCallback(int x, int y);
    virtual void KeyboardCallback(int key, int scancode, int action, int mods);
    virtual void KeyboardCharacterCallback(unsigned char key);
    virtual void MouseButtonCallback(int button, int action, int mods);
    virtual void MouseWheelCallback(int delta);

    void InitGui(u32 subpass_id = 0);
    void FitCamera(const glm::vec3& box_min, const glm::vec3& box_max, bool animated = false);

    bool IsMinimized(bool sleep_if_so);

    void SetupGlfwCallbacks();

    // Simple Getters

    vk::Instance       instance() { return instance_; }
    vk::Device         device() { return device_; }
    vk::PhysicalDevice gpu() { return gpu_; }
    vk::Queue          queue() { return queue_; }
    u32                queue_family() const { return graphics_queue_index_; }
    vk::CommandPool    cmd_pool() { return cmd_pool_; }
    vk::RenderPass     render_pass() { return render_pass_; }
    vk::Extent2D       window_size() { return window_size_; }
    vk::PipelineCache  pipeline_cache() { return pipeline_cache_; }
    vk::SurfaceKHR     surface() { return surface_; }
    vk::Format         color_format() const { return color_format_; }
    vk::Format         depth_format() const { return depth_format_; }

    const std::vector<vk::Framebuffer>&   framebuffers() { return framebuffers_; }
    const std::vector<vk::CommandBuffer>& cmd_buffers() { return cmd_buffers_; }

    u32 CurrentFrameIndex() const { return swapchain_.ActiveImageIndex(); }

  protected:
    vk::Instance       instance_;
    vk::Device         device_;
    vk::SurfaceKHR     surface_;
    vk::PhysicalDevice gpu_;
    vk::Queue          queue_;
    u32                graphics_queue_index_ = VK_QUEUE_FAMILY_IGNORED;
    vk::CommandPool    cmd_pool_;

    vkpbr::SwapChain               swapchain_;
    std::vector<vk::Framebuffer>   framebuffers_;
    std::vector<vk::CommandBuffer> cmd_buffers_;
    std::vector<vk::Fence>         wait_fences_;
    vk::Image                      depth_image_;
    vk::DeviceMemory               depth_memory_;
    vk::ImageView                  depth_view_;
    vk::RenderPass                 render_pass_;
    vk::Extent2D                   window_size_{0, 0};
    vk::PipelineCache              pipeline_cache_;
    GLFWwindow*                    window_ = nullptr;

    vk::Format color_format_ = vk::Format::eB8G8R8A8Unorm;
    vk::Format depth_format_ = vk::Format::eUndefined;

    u32 GetMemoryTypeIndex(u32 type_bits, vk::MemoryPropertyFlags properties) const;

    void UiDisplayHelp();
    bool show_help_ = false;

    CameraNavigator::Inputs          pressed_buttons_;
    std::shared_ptr<CameraNavigator> camera_navigator_;
    InertiaCamera                    inertia_camera_;
};

}  // namespace vkpbr

#endif  // !NVCOPY_VK_APPBASE_H_
