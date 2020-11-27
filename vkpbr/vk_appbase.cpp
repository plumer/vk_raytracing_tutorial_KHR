#include "vk_appbase.h"

#include "imgui/imgui.h"
#include "imgui/imgui_impl_vk.h"
#include "logging.h"

#ifdef __linux__
#include <unistd.h>
#endif

namespace {
constexpr float kKeyTau    = 0.10f;
constexpr float kCameraTau = 0.03f;
constexpr float kMoveStep  = 0.2f;

// Callback dispatchers
static void WindowSizeCallbackDispatcher(GLFWwindow* window, int w, int h) {
    auto app = reinterpret_cast<vkpbr::AppBase*>(glfwGetWindowUserPointer(window));
    app->WindowResizeCallback(w, h);
}
static void MouseButtonCallbackDispatcher(GLFWwindow* window, int button, int action, int mods) {
    auto app = reinterpret_cast<vkpbr::AppBase*>(glfwGetWindowUserPointer(window));
    app->MouseButtonCallback(button, action, mods);
}
static void CursorPosCallbackDispatcher(GLFWwindow* window, double x, double y) {
    auto app = reinterpret_cast<vkpbr::AppBase*>(glfwGetWindowUserPointer(window));
    app->MouseMotionCallback(x, y);
}
static void ScrollCallbackDispatcher(GLFWwindow* window, double x, double y) {
    auto app = reinterpret_cast<vkpbr::AppBase*>(glfwGetWindowUserPointer(window));
    // TODO(zixun): can we make use of x scroll?
    app->MouseWheelCallback(cast_i32(y));
}
static void KeyboardCallbackDispatcher(GLFWwindow* window, int key, int scancode, int action,
                                       int mods) {
    auto app = reinterpret_cast<vkpbr::AppBase*>(glfwGetWindowUserPointer(window));
    app->KeyboardCallback(key, scancode, action, mods);
}
static void CharacterCallbackDispatcher(GLFWwindow* window,unsigned int key) {
    auto app = reinterpret_cast<vkpbr::AppBase*>(glfwGetWindowUserPointer(window));
    app->KeyboardCharacterCallback(key);
}
static void DropCallbackDispatcher(GLFWwindow* window, int count, const char *paths[]) {
    auto app = reinterpret_cast<vkpbr::AppBase*>(glfwGetWindowUserPointer(window));
    ;
}

}  // namespace

namespace vkpbr {

void AppBase::Setup(const vk::Instance& instance, const vk::Device& device,
                    const vk::PhysicalDevice& gpu, u32 graphics_queue_index) {
    vk::DynamicLoader         loader;
    PFN_vkGetInstanceProcAddr vkGetInstanceProcAddr =
        loader.getProcAddress<PFN_vkGetInstanceProcAddr>("vkGetInstanceProcAddr");
    VULKAN_HPP_DEFAULT_DISPATCHER.init(vkGetInstanceProcAddr);
    VULKAN_HPP_DEFAULT_DISPATCHER.init(instance);
    VULKAN_HPP_DEFAULT_DISPATCHER.init(device);

    instance_             = instance;
    device_               = device;
    gpu_                  = gpu;
    graphics_queue_index_ = graphics_queue_index;
    queue_                = device_.getQueue(graphics_queue_index_, 0);

    cmd_pool_ = device_.createCommandPool(
        {vk::CommandPoolCreateFlagBits::eResetCommandBuffer, graphics_queue_index});
    pipeline_cache_ = device_.createPipelineCache({});
}

void AppBase::UnloadContext() {
    device_.waitIdle();
    if (ImGui::GetCurrentContext() != nullptr) {
        ImGui::ShutdownVK();
        ImGui::DestroyContext();
    }

    device_.destroyRenderPass(render_pass_);
    device_.destroyImageView(depth_view_);
    device_.destroyImage(depth_image_);
    device_.freeMemory(depth_memory_);
    device_.destroyPipelineCache(pipeline_cache_);

    for (u32 i = 0; i < swapchain_.NumImages(); ++i) {
        device_.destroyFence(wait_fences_[i]);
        device_.destroyFramebuffer(framebuffers_[i]);
        device_.freeCommandBuffers(cmd_pool_, cmd_buffers_[i]);
    }
    swapchain_.Deinit();

    device_.destroyCommandPool(cmd_pool_);
    if (surface_)
        instance_.destroySurfaceKHR(surface_);
}

vk::SurfaceKHR AppBase::AcquireSurface(const vk::Instance& instance, GLFWwindow* window) {
    CHECK(instance);
    window_ = window;
    VkSurfaceKHR surface_c;
    VkResult     error = glfwCreateWindowSurface(instance, window, nullptr, &surface_c);
    CHECK_EQ(error, VK_SUCCESS) << "Can't create a window surface";
    surface_ = surface_c;
    return surface_;
}

void AppBase::MakeSurface(const vk::SurfaceKHR& surface, u32 width, u32 height,
                          vk::Format color_format, vk::Format depth_format) {
    window_size_  = vk::Extent2D(width, height);
    depth_format_ = depth_format;
    color_format_ = color_format;

    swapchain_.Init(device_, gpu_, queue_, graphics_queue_index_, surface, color_format);
    swapchain_.Update(width, height);
    color_format_ = swapchain_.Format();

    // Creates synchronization Primitives.
    wait_fences_.resize(swapchain_.NumImages());
    for (auto& fence : wait_fences_) {
        fence = device_.createFence({vk::FenceCreateFlagBits::eSignaled});
    }

    // Command buffers store a reference to the framebuffer inside the renderpass info.
    // Creates one per framebuffer for static usage without rebuilding them during each frame.
    cmd_buffers_ =
        device_.allocateCommandBuffers(vk::CommandBufferAllocateInfo()
                                           .setCommandPool(cmd_pool_)
                                           .setLevel(vk::CommandBufferLevel::ePrimary)
                                           .setCommandBufferCount(swapchain_.NumImages()));

    CHECK(camera_navigator_);
    camera_navigator_->SetWindowSize(window_size_.width, window_size_.height);

#ifndef NDEBUG
    for (size_t i = 0; i < cmd_buffers_.size(); ++i) {
        std::string name = std::string("vkpbr::AppBase::Surface") + std::to_string(i);

        auto debug_info = vk::DebugUtilsObjectNameInfoEXT()
                              .setObjectType(vk::ObjectType::eCommandBuffer)
                              .setObjectHandle(reinterpret_cast<const u64&>(cmd_buffers_[i]))
                              .setPObjectName(name.c_str());
        device_.setDebugUtilsObjectNameEXT(debug_info);
    }
#endif
}

void AppBase::MakeFrameBuffers() {
    for (auto framebuffer : framebuffers_) {
        device_.destroyFramebuffer(framebuffer);
    }

    // Creates a 2-attachment (color + depth) array.
    std::array<vk::ImageView, 2> attachments;

    auto framebuffer_ci = vk::FramebufferCreateInfo()
                              .setRenderPass(render_pass_)
                              .setWidth(window_size_.width)
                              .setHeight(window_size_.height)
                              .setLayers(1)
                              .setAttachmentCount(2)
                              .setPAttachments(attachments.data());

    // Creates framebuffer one-by-one for each swapchain image.
    // Every framebuffer views into one separate swapchain image and the shared depth image.
    framebuffers_.resize(swapchain_.NumImages());
    for (u32 i = 0; i < swapchain_.NumImages(); ++i) {
        attachments[0]   = swapchain_.GetImageView(i);
        attachments[1]   = depth_view_;
        framebuffers_[i] = device_.createFramebuffer(framebuffer_ci);
    }
#ifndef NDEBUG
    for (size_t fb_i = 0; fb_i < framebuffers_.size(); ++fb_i) {
        std::string name = std::string("vkpbr::AppBase::FB") + std::to_string(fb_i);

        auto debug_info = vk::DebugUtilsObjectNameInfoEXT()
                              .setObjectType(vk::ObjectType::eFramebuffer)
                              .setObjectHandle(reinterpret_cast<const u64&>(framebuffers_[fb_i]))
                              .setPObjectName(name.c_str());
        device_.setDebugUtilsObjectNameEXT(debug_info);
    }
#endif  // !NDEBUG
}

void AppBase::MakeRenderPass() {
    if (render_pass_)
        device_.destroyRenderPass(render_pass_);

    std::array<vk::AttachmentDescription, 2> attachments;
    attachments[0]
        .setFormat(color_format_)
        .setLoadOp(vk::AttachmentLoadOp::eClear)
        .setFinalLayout(vk::ImageLayout::ePresentSrcKHR);
    attachments[1]
        .setFormat(depth_format_)
        .setLoadOp(vk::AttachmentLoadOp::eClear)
        .setStencilLoadOp(vk::AttachmentLoadOp::eClear)
        .setFinalLayout(vk::ImageLayout::eDepthStencilAttachmentOptimal);

    const auto color_ref = vk::AttachmentReference(0, vk::ImageLayout::eColorAttachmentOptimal);
    const auto depth_ref =
        vk::AttachmentReference(1, vk::ImageLayout::eDepthStencilAttachmentOptimal);

    auto subpass_dependencies =
        vk::SubpassDependency()
            .setSrcSubpass(VK_SUBPASS_EXTERNAL)
            .setDstSubpass(0)
            .setSrcStageMask(vk::PipelineStageFlagBits::eBottomOfPipe)
            .setDstStageMask(vk::PipelineStageFlagBits::eColorAttachmentOutput)
            .setSrcAccessMask(vk::AccessFlagBits::eMemoryRead)
            .setDstAccessMask(vk::AccessFlagBits::eColorAttachmentRead
                              | vk::AccessFlagBits::eColorAttachmentWrite)
            .setDependencyFlags(vk::DependencyFlagBits::eByRegion);

    auto subpass_description = vk::SubpassDescription()
                                   .setPipelineBindPoint(vk::PipelineBindPoint::eGraphics)
                                   .setColorAttachmentCount(1)
                                   .setPColorAttachments(&color_ref)
                                   .setPDepthStencilAttachment(&depth_ref);

    auto render_pass_ci = vk::RenderPassCreateInfo()
                              .setAttachmentCount(cast_u32(attachments.size()))
                              .setPAttachments(attachments.data())
                              .setSubpassCount(1)
                              .setPSubpasses(&subpass_description)
                              .setDependencyCount(1)
                              .setPDependencies(&subpass_dependencies);

    try {
        render_pass_ = device_.createRenderPass(render_pass_ci);
    } catch (std::exception& e) { LOG(FATAL) << "Can't create render pass: " << e.what(); }
#ifndef NDEBUG
    auto debug_info = vk::DebugUtilsObjectNameInfoEXT()
                          .setObjectType(vk::ObjectType::eRenderPass)
                          .setObjectHandle(reinterpret_cast<const u64&>(render_pass_))
                          .setPObjectName("vkpbr::AppBase");
    device_.setDebugUtilsObjectNameEXT(debug_info);
#endif  // !NDEBUG
}

void AppBase::MakeDepthBuffer() {
    if (depth_view_)
        device_.destroyImageView(depth_view_);
    if (depth_image_)
        device_.destroyImage(depth_image_);
    if (depth_memory_)
        device_.freeMemory(depth_memory_);

    // Step 1: Collects depth information and creates the depth image.
    const auto kAspect = vk::ImageAspectFlagBits::eDepth | vk::ImageAspectFlagBits::eStencil;
    auto       depth_stencil_ci = vk::ImageCreateInfo()
                                .setImageType(vk::ImageType::e2D)
                                .setExtent(vk::Extent3D{window_size_.width, window_size_.height, 1})
                                .setFormat(depth_format_)
                                .setMipLevels(1)
                                .setArrayLayers(1)
                                .setUsage(vk::ImageUsageFlagBits::eDepthStencilAttachment
                                          | vk::ImageUsageFlagBits::eTransferSrc);
    depth_image_ = device_.createImage(depth_stencil_ci);

    // Step 2: Allocates the memory and binds it with the image.
    auto mem_requirements = device_.getImageMemoryRequirements(depth_image_);
    u32  mem_type_index   = GetMemoryTypeIndex(mem_requirements.memoryTypeBits,
                                            vk::MemoryPropertyFlagBits::eDeviceLocal);
    auto mem_alloc_info   = vk::MemoryAllocateInfo()
                              .setAllocationSize(mem_requirements.size)
                              .setMemoryTypeIndex(mem_type_index);
    depth_memory_ = device_.allocateMemory(mem_alloc_info);

    device_.bindImageMemory(depth_image_, depth_memory_, 0);

    // Creates an image barrier to change the layout from undefined to DepthStencilAttachmentOptimal.
    auto cmd_buffer_alloc_info = vk::CommandBufferAllocateInfo()
                                     .setCommandPool(cmd_pool_)
                                     .setLevel(vk::CommandBufferLevel::ePrimary)
                                     .setCommandBufferCount(1);
    vk::CommandBuffer cmd_buffer = device_.allocateCommandBuffers(cmd_buffer_alloc_info)[0];

    cmd_buffer.begin(vk::CommandBufferBeginInfo(vk::CommandBufferUsageFlagBits::eOneTimeSubmit));
    {
        // Puts barrier on top, and inside setup command buffer.
        auto subresource_range =
            vk::ImageSubresourceRange().setAspectMask(kAspect).setLevelCount(1).setLayerCount(1);
        auto image_memory_barrier =
            vk::ImageMemoryBarrier()
                .setOldLayout(vk::ImageLayout::eUndefined)
                .setNewLayout(vk::ImageLayout::eDepthStencilReadOnlyOptimal)
                .setImage(depth_image_)
                .setSubresourceRange(subresource_range)
                .setSrcAccessMask({})
                .setDstAccessMask(vk::AccessFlagBits::eDepthStencilAttachmentWrite);
        const auto kSrcStageMask = vk::PipelineStageFlagBits::eTopOfPipe;
        const auto kDstStageMask = vk::PipelineStageFlagBits::eEarlyFragmentTests;
        cmd_buffer.pipelineBarrier(kSrcStageMask, kDstStageMask, vk::DependencyFlags(), nullptr,
                                   nullptr, image_memory_barrier);
    }
    cmd_buffer.end();

    queue_.submit(vk::SubmitInfo(0, nullptr, nullptr, 1, &cmd_buffer), vk::Fence());
    queue_.waitIdle();
    device_.freeCommandBuffers(cmd_pool_, cmd_buffer);

    // Step3: Sets up the depth image view.
    auto depth_stencil_view_ci = vk::ImageViewCreateInfo()
                                     .setViewType(vk::ImageViewType::e2D)
                                     .setFormat(depth_format_)
                                     .setSubresourceRange({kAspect, 0, 1, 0, 1})
                                     .setImage(depth_image_);
    depth_view_ = device_.createImageView(depth_stencil_view_ci);
}

void AppBase::PrepareFrame() {
    if (!swapchain_.Acquire()) {
        LOG(FATAL) << "Can acquire the next image from the swap chain";
    }
    // Waits until the command buffer has finished execution before being used again.
    u32 image_index = swapchain_.ActiveImageIndex();
    while (device_.waitForFences(wait_fences_[image_index], VK_TRUE, 10000)
           == vk::Result::eTimeout) {}
}

void AppBase::SubmitFrame() {
    u32 image_index = swapchain_.ActiveImageIndex();
    device_.resetFences(wait_fences_[image_index]);

    const u32 kDeviceMask     = 0b0000'00001;
    const u32 kDeviceIndex[2] = {0, 1};

    auto device_group_submit_info = vk::DeviceGroupSubmitInfo()
                                        .setWaitSemaphoreCount(1)
                                        .setCommandBufferCount(1)
                                        .setPCommandBufferDeviceMasks(&kDeviceMask)
                                        .setSignalSemaphoreCount(1)
                                        .setPSignalSemaphoreDeviceIndices(kDeviceIndex)
                                        .setPWaitSemaphoreDeviceIndices(kDeviceIndex);

    auto semaphore_read  = swapchain_.ActiveReadSemaphore();
    auto semaphore_write = swapchain_.ActiveWrittenSemaphore();

    const vk::PipelineStageFlags kWaitStageMask = vk::PipelineStageFlagBits::eColorAttachmentOutput;
    // The submit info structure specifies a commadn buffer queue submission batch.
    auto submit_info = vk::SubmitInfo()
                           .setPWaitDstStageMask(&kWaitStageMask)
                           .setPWaitSemaphores(&semaphore_read)
                           .setWaitSemaphoreCount(1)
                           .setPSignalSemaphores(&semaphore_write)
                           .setSignalSemaphoreCount(1)
                           .setPCommandBuffers(&cmd_buffers_[image_index])
                           .setCommandBufferCount(1)
                           .setPNext(&device_group_submit_info);

    // Submits to the graphics queue passing a wait fence.
    queue_.submit(submit_info, wait_fences_[image_index]);

    // Presents the frame.
    swapchain_.Present(queue_);
}

void AppBase::SetViewport(const vk::CommandBuffer& cmd_buffer) {
    cmd_buffer.setViewport(0, {vk::Viewport(0.f, 0.f, cast_f32(window_size_.width),
                                            cast_f32(window_size_.height), 0.0f, 1.0f)});
    cmd_buffer.setScissor(0, {vk::Rect2D().setOffset({0, 0}).setExtent(window_size_)});
}

void AppBase::WindowResizeCallback(int width, int height) {
    if (width == 0 || height == 0)
        return;
    window_size_ = vk::Extent2D(width, height);

    // Updates ImGui and camera.
    if (ImGui::GetCurrentContext() != nullptr) {
        auto& imgui_io       = ImGui::GetIO();
        imgui_io.DisplaySize = ImVec2(cast_f32(width), cast_f32(height));
    }

    camera_navigator_->SetWindowSize(width, height);

    device_.waitIdle();
    queue_.waitIdle();

    swapchain_.Update(width, height);
    // TODO(zixun): calls window resize callback for inherited class.
    MakeDepthBuffer();
    MakeFrameBuffers();
}

void AppBase::MouseMotionCallback(int x, int y) {
    if (ImGui::GetCurrentContext() != nullptr && ImGui::GetIO().WantCaptureMouse)
        return;

    glm::vec2 old_mouse_pos = camera_navigator_->MousePosition();
    int       old_x         = cast_i32(old_mouse_pos.x);
    int       old_y         = cast_i32(old_mouse_pos.y);
    if (pressed_buttons_.left_mouse || pressed_buttons_.middle_mouse
        || pressed_buttons_.right_mouse)
        camera_navigator_->MouseMove(x, y, pressed_buttons_);

    constexpr float kKeyTau    = 0.10f;
    inertia_camera_.SetTau(kKeyTau);
    const int dx = cast_i32(x - old_x), dy = cast_i32(y - old_y);
    if (pressed_buttons_.left_mouse) {
        inertia_camera_.RotateH(2 * dx / cast_f32(window_size_.width));
        inertia_camera_.RotateV(2 * dy / cast_f32(window_size_.height));
    }
    if (pressed_buttons_.middle_mouse) {
        inertia_camera_.RotateH(2 * dx / cast_f32(window_size_.width), /*panning=*/true);
        inertia_camera_.RotateV(2 * dy / cast_f32(window_size_.height), /*panning=*/true);
    }
    if (pressed_buttons_.right_mouse) {
        inertia_camera_.RotateH(2 * dx / cast_f32(window_size_.width), pressed_buttons_.ctrl_key);
        inertia_camera_.Move(-2 * dy / cast_f32(window_size_.height), pressed_buttons_.ctrl_key);
    }
}

void AppBase::KeyboardCallback(int key, int scancode, int action, int modes) {
    const bool kCapture =
        ImGui::GetCurrentContext() != nullptr && ImGui::GetIO().WantCaptureKeyboard;
    bool pressed = action != GLFW_RELEASE;

    switch (key) {
        case GLFW_KEY_LEFT_CONTROL:
            pressed_buttons_.ctrl_key = pressed;
            break;
        case GLFW_KEY_LEFT_SHIFT:
            pressed_buttons_.shift_key = pressed;
            break;
        case GLFW_KEY_LEFT_ALT:
            pressed_buttons_.alt_key = pressed;
            break;
    }

    // Stop actions if no keys are pressed or ImGUI wants to use the key.
    if (!pressed || kCapture)
        return;


    inertia_camera_.SetTau(kKeyTau);
    switch (key) {
        case GLFW_KEY_ESCAPE:
            glfwSetWindowShouldClose(window_, true);
            break;
        case GLFW_KEY_LEFT:
            inertia_camera_.RotateH(kMoveStep, pressed_buttons_.ctrl_key);
            break;
        case GLFW_KEY_UP:
            inertia_camera_.RotateV(kMoveStep, pressed_buttons_.ctrl_key);
            break;
        case GLFW_KEY_RIGHT:
            inertia_camera_.RotateH(-kMoveStep, pressed_buttons_.ctrl_key);
            break;
        case GLFW_KEY_DOWN:
            inertia_camera_.RotateH(-kMoveStep, pressed_buttons_.ctrl_key);
            break;
        case GLFW_KEY_PAGE_UP:
            inertia_camera_.Move(kMoveStep, pressed_buttons_.ctrl_key);
            break;
        case GLFW_KEY_PAGE_DOWN:
            inertia_camera_.Move(-kMoveStep, pressed_buttons_.ctrl_key);
        default:
            break;
    }
}

void AppBase::KeyboardCharacterCallback(unsigned char key) {
    if (ImGui::GetCurrentContext() != nullptr && ImGui::GetIO().WantCaptureKeyboard)
        return;

    if (key == 'h' || key == '?')
        show_help_ = !show_help_;
}

void AppBase::MouseButtonCallback(int button, int action, int mods) {
    if (ImGui::GetCurrentContext() != nullptr && ImGui::GetIO().WantCaptureMouse)
        return;

    double x, y;
    glfwGetCursorPos(window_, &x, &y);
    camera_navigator_->SetMousePosition(cast_i32(x), cast_i32(y));

    pressed_buttons_.left_mouse   = (action == GLFW_PRESS) && (button == GLFW_MOUSE_BUTTON_LEFT);
    pressed_buttons_.middle_mouse = (action == GLFW_PRESS) && (button == GLFW_MOUSE_BUTTON_MIDDLE);
    pressed_buttons_.right_mouse  = (action == GLFW_PRESS) && (button == GLFW_MOUSE_BUTTON_RIGHT);
}

void AppBase::MouseWheelCallback(int delta) {
    if (ImGui::GetCurrentContext() != nullptr && ImGui::GetIO().WantCaptureMouse)
        return;
    camera_navigator_->Wheel(delta > 0 ? 1 : -1, pressed_buttons_);
    inertia_camera_.SetTau(kKeyTau);
    inertia_camera_.Move(delta > 0 ? kMoveStep : -kMoveStep, pressed_buttons_.ctrl_key);
}

void AppBase::InitGui(u32 subpass_id) {
    CHECK(render_pass_) << "Render pass must be valid";

    ImGui::CreateContext();
    ImGui::InitVK(device_, gpu_, queue_, graphics_queue_index_, render_pass_, subpass_id);
    ImGui::GetIO().IniFilename = nullptr;
}

void AppBase::FitCamera(const glm::vec3& box_min, const glm::vec3& box_max, bool animated) {
    camera_navigator_->Fit(box_min, box_max, animated);
}

bool AppBase::IsMinimized(bool sleep_if_so) {
    int width, height;
    glfwGetWindowSize(window_, &width, &height);
    bool minimized = width * height == 0;
    if (minimized && sleep_if_so) {
#ifdef _WIN32
        Sleep(50); // milliseconds
#else
        usleep(50);
#endif  // _WIN32
    }
    return minimized;
}

void AppBase::SetupGlfwCallbacks() {
    CHECK(window_);
    glfwSetWindowUserPointer(window_, this);

    glfwSetKeyCallback(window_, KeyboardCallbackDispatcher);
    glfwSetCharCallback(window_, CharacterCallbackDispatcher);
    glfwSetCursorPosCallback(window_, CursorPosCallbackDispatcher);
    glfwSetMouseButtonCallback(window_, MouseButtonCallbackDispatcher);
    glfwSetScrollCallback(window_, ScrollCallbackDispatcher);
    glfwSetWindowSizeCallback(window_, WindowSizeCallbackDispatcher);
    glfwSetDropCallback(window_, DropCallbackDispatcher);
}



u32 AppBase::GetMemoryTypeIndex(u32 type_bits, vk::MemoryPropertyFlags properties) const {
    auto device_memory_properties = gpu_.getMemoryProperties();
    for (u32 i = 0; i < device_memory_properties.memoryTypeCount; ++i) {
        bool i_th_bit_matched = (type_bits & (i << i)) > 0;
        bool i_th_property_satisfied =
            (device_memory_properties.memoryTypes[i].propertyFlags & properties) == properties;
        if (i_th_bit_matched && i_th_property_satisfied) {
            return i;
        }
    }
    LOG(FATAL) << "Unable to find memory type " << vk::to_string(properties);
    return ~0u;
}

void AppBase::UiDisplayHelp() {
    static const char* kHelpText =
        "LMB: rotate around the target\n"
        "RMB: Dolly in/out\n"
        "MMB: Pan along view plane\n"
        "LMB + Shift: Dolly in/out\n"
        "LMB + Ctrl: Pan\n"
        "LMB + Alt: Look aroundPan\n"
        "Mouse wheel: Dolly in/out\n"
        "Mouse wheel + Shift: Zoom in/out\n";
    if (show_help_) {
        ImGui::BeginChild("Help", ImVec2(370, 120), true);
        ImGui::Text("%s", kHelpText);
        ImGui::EndChild();
    }
}

}  // namespace vkpbr
