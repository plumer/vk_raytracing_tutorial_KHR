#include "vk_appbase.h"
#include "imgui.h"
#include "imgui_impl_vk.h"
#include "vk_utils.h"

#include "logging.h"
#include "imgui_helper.h"

//--------------------------------------------------------------------------------------------------
// Setup the low level Vulkan for various operations
//

namespace vkpbr {

void AppBase::setup(const vk::Instance& instance, const vk::Device& device,
                    const vk::PhysicalDevice& physicalDevice, uint32_t graphicsQueueIndex)
{
    // Initialize function pointers
    vk::DynamicLoader         dl;
    PFN_vkGetInstanceProcAddr vkGetInstanceProcAddr =
        dl.getProcAddress<PFN_vkGetInstanceProcAddr>("vkGetInstanceProcAddr");
    VULKAN_HPP_DEFAULT_DISPATCHER.init(vkGetInstanceProcAddr);
    VULKAN_HPP_DEFAULT_DISPATCHER.init(instance);
    VULKAN_HPP_DEFAULT_DISPATCHER.init(device);

    m_instance           = instance;
    m_device             = device;
    m_physicalDevice     = physicalDevice;
    m_graphicsQueueIndex = graphicsQueueIndex;
    m_queue              = m_device.getQueue(m_graphicsQueueIndex, 0);
    m_cmdPool            = m_device.createCommandPool(
        {vk::CommandPoolCreateFlagBits::eResetCommandBuffer, graphicsQueueIndex});
    m_pipelineCache = device.createPipelineCache(vk::PipelineCacheCreateInfo());

    // ImGuiH::SetCameraJsonFile(PROJECT_NAME);
}

//--------------------------------------------------------------------------------------------------
// To call on exit
//

void AppBase::destroy()
{
    m_device.waitIdle();

    if (ImGui::GetCurrentContext() != nullptr) {
        ImGui::ShutdownVK();
        ImGui::DestroyContext();
    }

    m_device.destroy(m_renderPass);
    m_device.destroy(m_depthView);
    m_device.destroy(m_depthImage);
    m_device.freeMemory(m_depthMemory);
    m_device.destroy(m_pipelineCache);

    for (uint32_t i = 0; i < m_swapChain.getImageCount(); i++) {
        m_device.destroy(m_waitFences[i]);
        m_device.destroy(m_framebuffers[i]);
        m_device.freeCommandBuffers(m_cmdPool, m_commandBuffers[i]);
    }
    m_swapChain.deinit();

    m_device.destroy(m_cmdPool);
    if (m_surface)
        m_instance.destroySurfaceKHR(m_surface);
}

//--------------------------------------------------------------------------------------------------
// Return the surface "screen" for the display
//

VkSurfaceKHR AppBase::getVkSurface(const vk::Instance& instance, GLFWwindow* window)
{
    assert(instance);
    m_window = window;

    VkSurfaceKHR surface{};
    VkResult     err = glfwCreateWindowSurface(instance, window, nullptr, &surface);
    if (err != VK_SUCCESS) {
        assert(!"Failed to create a Window surface");
    }
    m_surface = surface;

    return surface;
}

//--------------------------------------------------------------------------------------------------
// Creating the surface for rendering
//

void AppBase::createSwapchain(const vk::SurfaceKHR& surface, uint32_t width, uint32_t height,
                              vk::Format colorFormat, vk::Format depthFormat, bool vsync)
{
    m_size        = vk::Extent2D(width, height);
    m_depthFormat = depthFormat;
    m_colorFormat = colorFormat;
    m_vsync       = vsync;

    m_swapChain.init(m_device, m_physicalDevice, m_queue, m_graphicsQueueIndex, surface,
                     static_cast<VkFormat>(colorFormat));
    m_size        = m_swapChain.update(m_size.width, m_size.height, vsync);
    m_colorFormat = static_cast<vk::Format>(m_swapChain.getFormat());

    // Create Synchronization Primitives
    m_waitFences.resize(m_swapChain.getImageCount());
    for (auto& fence : m_waitFences) {
        fence = m_device.createFence({vk::FenceCreateFlagBits::eSignaled});
    }

    // Command buffers store a reference to the frame buffer inside their render pass info
    // so for static usage without having to rebuild them each frame, we use one per frame
    // buffer
    m_commandBuffers = m_device.allocateCommandBuffers(
        {m_cmdPool, vk::CommandBufferLevel::ePrimary, m_swapChain.getImageCount()});

#ifdef _DEBUG
    for (size_t i = 0; i < m_commandBuffers.size(); i++) {
        std::string name = std::string("AppBase") + std::to_string(i);
        m_device.setDebugUtilsObjectNameEXT({vk::ObjectType::eCommandBuffer,
                                             reinterpret_cast<const uint64_t&>(m_commandBuffers[i]),
                                             name.c_str()});
    }
#endif  // _DEBUG

    // Setup camera
    // CameraManip.setWindowSize(m_size.width, m_size.height);
    camera_->SetWindowSize(m_size.width, m_size.height);
}

//--------------------------------------------------------------------------------------------------
// Create the framebuffers in which the image will be rendered
// - Swapchain need to be created before calling this
//

void AppBase::createFrameBuffers()
{
    // Recreate the frame buffers
    for (auto framebuffer : m_framebuffers) {
        m_device.destroy(framebuffer);
    }

    // Array of attachment (color, depth)
    std::array<vk::ImageView, 2> attachments;

    // Create frame buffers for every swap chain image
    vk::FramebufferCreateInfo framebufferCreateInfo;
    framebufferCreateInfo.renderPass      = m_renderPass;
    framebufferCreateInfo.attachmentCount = 2;
    framebufferCreateInfo.width           = m_size.width;
    framebufferCreateInfo.height          = m_size.height;
    framebufferCreateInfo.layers          = 1;
    framebufferCreateInfo.pAttachments    = attachments.data();

    // Create frame buffers for every swap chain image
    m_framebuffers.resize(m_swapChain.getImageCount());
    for (uint32_t i = 0; i < m_swapChain.getImageCount(); i++) {
        attachments[0]    = m_swapChain.getImageView(i);
        attachments[1]    = m_depthView;
        m_framebuffers[i] = m_device.createFramebuffer(framebufferCreateInfo);
    }


#ifdef _DEBUG
    for (size_t i = 0; i < m_framebuffers.size(); i++) {
        std::string name = std::string("AppBase") + std::to_string(i);
        m_device.setDebugUtilsObjectNameEXT({vk::ObjectType::eFramebuffer,
                                             reinterpret_cast<const uint64_t&>(m_framebuffers[i]),
                                             name.c_str()});
    }
#endif  // _DEBUG
}

//--------------------------------------------------------------------------------------------------
// Creating a default render pass, very simple one.
// Other examples will mostly override this one.
//

void AppBase::createRenderPass()
{
    if (m_renderPass) {
        m_device.destroy(m_renderPass);
    }

    std::array<vk::AttachmentDescription, 2> attachments;
    // Color attachment
    attachments[0].setFormat(m_colorFormat);
    attachments[0].setLoadOp(vk::AttachmentLoadOp::eClear);
    attachments[0].setFinalLayout(vk::ImageLayout::ePresentSrcKHR);

    // Depth attachment
    attachments[1].setFormat(m_depthFormat);
    attachments[1].setLoadOp(vk::AttachmentLoadOp::eClear);
    attachments[1].setStencilLoadOp(vk::AttachmentLoadOp::eClear);
    attachments[1].setFinalLayout(vk::ImageLayout::eDepthStencilAttachmentOptimal);

    // One color, one depth
    const vk::AttachmentReference colorReference{0, vk::ImageLayout::eColorAttachmentOptimal};
    const vk::AttachmentReference depthReference{1,
                                                 vk::ImageLayout::eDepthStencilAttachmentOptimal};

    std::array<vk::SubpassDependency, 1> subpassDependencies;
    // Transition from final to initial (VK_SUBPASS_EXTERNAL refers to all commands executed
    // outside of the actual renderpass)
    subpassDependencies[0].setSrcSubpass(VK_SUBPASS_EXTERNAL);
    subpassDependencies[0].setDstSubpass(0);
    subpassDependencies[0].setSrcStageMask(vk::PipelineStageFlagBits::eBottomOfPipe);
    subpassDependencies[0].setDstStageMask(vk::PipelineStageFlagBits::eColorAttachmentOutput);
    subpassDependencies[0].setSrcAccessMask(vk::AccessFlagBits::eMemoryRead);
    subpassDependencies[0].setDstAccessMask(vk::AccessFlagBits::eColorAttachmentRead
                                            | vk::AccessFlagBits::eColorAttachmentWrite);
    subpassDependencies[0].setDependencyFlags(vk::DependencyFlagBits::eByRegion);

    vk::SubpassDescription subpassDescription;
    subpassDescription.setPipelineBindPoint(vk::PipelineBindPoint::eGraphics);
    subpassDescription.setColorAttachmentCount(1);
    subpassDescription.setPColorAttachments(&colorReference);
    subpassDescription.setPDepthStencilAttachment(&depthReference);

    vk::RenderPassCreateInfo renderPassInfo;
    renderPassInfo.setAttachmentCount(static_cast<uint32_t>(attachments.size()));
    renderPassInfo.setPAttachments(attachments.data());
    renderPassInfo.setSubpassCount(1);
    renderPassInfo.setPSubpasses(&subpassDescription);
    renderPassInfo.setDependencyCount(static_cast<uint32_t>(subpassDependencies.size()));
    renderPassInfo.setPDependencies(subpassDependencies.data());

    m_renderPass = m_device.createRenderPass(renderPassInfo);

#ifdef _DEBUG
    m_device.setDebugUtilsObjectNameEXT(
        {vk::ObjectType::eRenderPass, reinterpret_cast<const uint64_t&>(m_renderPass), "AppBase"});
#endif  // _DEBUG
}

//--------------------------------------------------------------------------------------------------
// Creating an image to be used as depth buffer
//

void AppBase::createDepthBuffer()
{
    if (m_depthView)
        m_device.destroy(m_depthView);
    if (m_depthImage)
        m_device.destroy(m_depthImage);
    if (m_depthMemory)
        m_device.freeMemory(m_depthMemory);

    // Depth information
    const vk::ImageAspectFlags aspect =
        vk::ImageAspectFlagBits::eDepth | vk::ImageAspectFlagBits::eStencil;
    vk::ImageCreateInfo depthStencilCreateInfo;
    depthStencilCreateInfo.setImageType(vk::ImageType::e2D);
    depthStencilCreateInfo.setExtent(vk::Extent3D{m_size.width, m_size.height, 1});
    depthStencilCreateInfo.setFormat(m_depthFormat);
    depthStencilCreateInfo.setMipLevels(1);
    depthStencilCreateInfo.setArrayLayers(1);
    depthStencilCreateInfo.setUsage(vk::ImageUsageFlagBits::eDepthStencilAttachment
                                    | vk::ImageUsageFlagBits::eTransferSrc);
    // Create the depth image
    m_depthImage = m_device.createImage(depthStencilCreateInfo);

    // Allocate the memory
    const vk::MemoryRequirements memReqs = m_device.getImageMemoryRequirements(m_depthImage);
    vk::MemoryAllocateInfo       memAllocInfo;
    memAllocInfo.allocationSize = memReqs.size;
    memAllocInfo.memoryTypeIndex =
        getMemoryType(memReqs.memoryTypeBits, vk::MemoryPropertyFlagBits::eDeviceLocal);
    m_depthMemory = m_device.allocateMemory(memAllocInfo);

    // Bind image and memory
    m_device.bindImageMemory(m_depthImage, m_depthMemory, 0);

    // Create an image barrier to change the layout from undefined to
    // DepthStencilAttachmentOptimal
    vk::CommandBuffer             cmdBuffer;
    vk::CommandBufferAllocateInfo cmdBufAllocateInfo;
    cmdBufAllocateInfo.commandPool        = m_cmdPool;
    cmdBufAllocateInfo.level              = vk::CommandBufferLevel::ePrimary;
    cmdBufAllocateInfo.commandBufferCount = 1;
    cmdBuffer                             = m_device.allocateCommandBuffers(cmdBufAllocateInfo)[0];
    cmdBuffer.begin(vk::CommandBufferBeginInfo{vk::CommandBufferUsageFlagBits::eOneTimeSubmit});

    // Put barrier on top, Put barrier inside setup command buffer
    vk::ImageSubresourceRange subresourceRange;
    subresourceRange.aspectMask = aspect;
    subresourceRange.levelCount = 1;
    subresourceRange.layerCount = 1;
    vk::ImageMemoryBarrier imageMemoryBarrier;
    imageMemoryBarrier.oldLayout               = vk::ImageLayout::eUndefined;
    imageMemoryBarrier.newLayout               = vk::ImageLayout::eDepthStencilAttachmentOptimal;
    imageMemoryBarrier.image                   = m_depthImage;
    imageMemoryBarrier.subresourceRange        = subresourceRange;
    imageMemoryBarrier.srcAccessMask           = vk::AccessFlags();
    imageMemoryBarrier.dstAccessMask           = vk::AccessFlagBits::eDepthStencilAttachmentWrite;
    const vk::PipelineStageFlags srcStageMask  = vk::PipelineStageFlagBits::eTopOfPipe;
    const vk::PipelineStageFlags destStageMask = vk::PipelineStageFlagBits::eEarlyFragmentTests;

    cmdBuffer.pipelineBarrier(srcStageMask, destStageMask, vk::DependencyFlags(), nullptr, nullptr,
                              imageMemoryBarrier);
    cmdBuffer.end();
    m_queue.submit(vk::SubmitInfo{0, nullptr, nullptr, 1, &cmdBuffer}, vk::Fence());
    m_queue.waitIdle();
    m_device.freeCommandBuffers(m_cmdPool, cmdBuffer);

    // Setting up the view
    vk::ImageViewCreateInfo depthStencilView;
    depthStencilView.setViewType(vk::ImageViewType::e2D);
    depthStencilView.setFormat(m_depthFormat);
    depthStencilView.setSubresourceRange({aspect, 0, 1, 0, 1});
    depthStencilView.setImage(m_depthImage);
    m_depthView = m_device.createImageView(depthStencilView);
}

//--------------------------------------------------------------------------------------------------
// Convenient function to call for submitting the rendering command
//

void AppBase::submitFrame()
{
    uint32_t imageIndex = m_swapChain.getActiveImageIndex();
    m_device.resetFences(m_waitFences[imageIndex]);

    // In case of using NVLINK
    const uint32_t                deviceMask  = m_useNvlink ? 0b0000'0011 : 0b0000'0001;
    const std::array<uint32_t, 2> deviceIndex = {0, 1};

    vk::DeviceGroupSubmitInfo deviceGroupSubmitInfo;
    deviceGroupSubmitInfo.setWaitSemaphoreCount(1);
    deviceGroupSubmitInfo.setCommandBufferCount(1);
    deviceGroupSubmitInfo.setPCommandBufferDeviceMasks(&deviceMask);
    deviceGroupSubmitInfo.setSignalSemaphoreCount(m_useNvlink ? 2 : 1);
    deviceGroupSubmitInfo.setPSignalSemaphoreDeviceIndices(deviceIndex.data());
    deviceGroupSubmitInfo.setPWaitSemaphoreDeviceIndices(deviceIndex.data());

    vk::Semaphore semaphoreRead  = m_swapChain.getActiveReadSemaphore();
    vk::Semaphore semaphoreWrite = m_swapChain.getActiveWrittenSemaphore();

    // Pipeline stage at which the queue submission will wait (via pWaitSemaphores)
    const vk::PipelineStageFlags waitStageMask = vk::PipelineStageFlagBits::eColorAttachmentOutput;
    // The submit info structure specifies a command buffer queue submission batch
    vk::SubmitInfo submitInfo;
    submitInfo.setPWaitDstStageMask(&waitStageMask);   // Pointer to the list of pipeline stages
                                                       // that the semaphore waits will occur at

    //submitInfo.setPWaitSemaphores(&semaphoreRead);     
    //submitInfo.setWaitSemaphoreCount(1);               // One wait semaphore
    // Semaphore(s) to wait upon before the submitted command buffer starts executing
    submitInfo.setWaitSemaphores(semaphoreRead);

    //submitInfo.setPSignalSemaphores(&semaphoreWrite);  
    //submitInfo.setSignalSemaphoreCount(1);             // One signal semaphore
    // Semaphore(s) to be signaled when command buffers have completed.
    submitInfo.setSignalSemaphores(semaphoreWrite);

    //submitInfo.setPCommandBuffers(&m_commandBuffers[imageIndex]);  
    //submitInfo.setCommandBufferCount(1);                           // One command buffer
    // Command buffers(s) to execute in this batch (submission).
    submitInfo.setCommandBuffers(m_commandBuffers[imageIndex]);
    submitInfo.setPNext(&deviceGroupSubmitInfo);

    // Submit to the graphics queue passing a wait fence
    m_queue.submit(submitInfo, m_waitFences[imageIndex]);

    // Presenting frame
    m_swapChain.present(m_queue);
}

//--------------------------------------------------------------------------------------------------
// When the pipeline is set for using dynamic, this becomes useful
//

void AppBase::setViewport(const vk::CommandBuffer& cmdBuf)
{
    cmdBuf.setViewport(0, {vk::Viewport(0.0f, 0.0f, static_cast<float>(m_size.width),
                                        static_cast<float>(m_size.height), 0.0f, 1.0f)});
    cmdBuf.setScissor(0, {{{0, 0}, {m_size.width, m_size.height}}});
}

//--------------------------------------------------------------------------------------------------
// Window callback when the it is resized
// - Destroy allocated frames, then rebuild them with the new size
// - Call onResize() of the derived class
//

void AppBase::onFramebufferSize(int w, int h)
{
    if (w == 0 || h == 0) {
        return;
    }

    // Update imgui
    if (ImGui::GetCurrentContext() != nullptr) {
        auto& imgui_io       = ImGui::GetIO();
        imgui_io.DisplaySize = ImVec2(static_cast<float>(w), static_cast<float>(h));
    }
    // Wait to finish what is currently drawing
    m_device.waitIdle();
    m_queue.waitIdle();

    // Request new swapschain image size
    m_size = m_swapChain.update(m_size.width, m_size.height, m_vsync);

    if (m_size.width != w || m_size.height != h) {
        LOG(WARNING) << Format("Requested size (%d, %d) is different from created size (%u, %u) ",
                               w, h, m_size.width, m_size.height);
    }

    // CameraManip.setWindowSize(m_size.width, m_size.height);
    // Invoking Sample callback
    onResize(m_size.width, m_size.height);
    // Recreating other resources
    createDepthBuffer();
    createFrameBuffers();
}

//--------------------------------------------------------------------------------------------------
// Window callback when a special key gets hit
// - Handling ImGui and a default camera
//


//--------------------------------------------------------------------------------------------------
// Window callback when the mouse move
// - Handling ImGui and a default camera
//

void AppBase::onMouseMotion(int x, int y)
{
    if (ImGui::GetCurrentContext() != nullptr && ImGui::GetIO().WantCaptureMouse) {
        return;
    }

    // if (m_inputs.lmb || m_inputs.rmb || m_inputs.mmb) {
    //    CameraManip.mouseMove(x, y, m_inputs);
    //}
    if (camera_inputs_.left_mouse || camera_inputs_.right_mouse || camera_inputs_.middle_mouse) {
        camera_->MouseMove(x, y, camera_inputs_);
    }
}

void AppBase::onKeyboard(int key, int scancode, int action, int mods)
{
    const bool capture =
        ImGui::GetCurrentContext() != nullptr && ImGui::GetIO().WantCaptureKeyboard;
    const bool pressed = action != GLFW_RELEASE;

    // Keeping track of the modifiers
    camera_inputs_.ctrl_key  = mods & GLFW_MOD_CONTROL;
    camera_inputs_.shift_key = mods & GLFW_MOD_SHIFT;
    camera_inputs_.alt_key   = mods & GLFW_MOD_ALT;

    // Remember all keys that are pressed for animating the camera when
    // many keys are pressed and stop when all keys are released.
    if (pressed) {
        m_keys.insert(key);
    } else {
        m_keys.erase(key);
    }

    // For all pressed keys - apply the action
    // CameraManip.keyMotion(0, 0, nvh::CameraManipulator::NoAction);
    camera_->Motion(0, 0, CameraNavigator::Actions::kNoAction);
    for (auto key : m_keys) {
        switch (key) {
            case GLFW_KEY_F10:
                m_show_gui = !m_show_gui;
                break;
            case GLFW_KEY_ESCAPE:
                glfwSetWindowShouldClose(m_window, 1);
                break;
            case GLFW_KEY_W:
                camera_->Motion(1.f, 0, CameraNavigator::Actions::kDolly);
                break;
            case GLFW_KEY_S:
                camera_->Motion(-1.f, 0, CameraNavigator::Actions::kDolly);
                break;
            case GLFW_KEY_A:
            case GLFW_KEY_LEFT:
                camera_->Motion(-1.f, 0, CameraNavigator::Actions::kPan);
                break;
            case GLFW_KEY_UP:
                camera_->Motion(0, 1, CameraNavigator::Actions::kPan);
                break;
            case GLFW_KEY_D:
            case GLFW_KEY_RIGHT:
                camera_->Motion(1.f, 0, CameraNavigator::Actions::kPan);
                break;
            case GLFW_KEY_DOWN:
                camera_->Motion(0, -1, CameraNavigator::Actions::kPan);
                break;
            default:
                break;
        }
    }
}

//--------------------------------------------------------------------------------------------------
// Window callback when a key gets hit
//

void AppBase::onKeyboardChar(unsigned char key)
{
    if (ImGui::GetCurrentContext() != nullptr && ImGui::GetIO().WantCaptureKeyboard) {
        return;
    }

    // Toggling vsync
    if (key == 'v') {
        m_vsync = !m_vsync;
        m_device.waitIdle();
        m_queue.waitIdle();
        m_swapChain.update(m_size.width, m_size.height, m_vsync);
        createFrameBuffers();
    }

    if (key == 'h' || key == '?')
        m_showHelp = !m_showHelp;
}

//--------------------------------------------------------------------------------------------------
// Window callback when the mouse button is pressed
// - Handling ImGui and a default camera
//

void AppBase::onMouseButton(int button, int action, int mods)
{
    (void)mods;

    double x, y;
    glfwGetCursorPos(m_window, &x, &y);
    //CameraManip.setMousePosition(static_cast<int>(x), static_cast<int>(y));
    camera_->SetMousePosition(cast_i32(x), cast_i32(y));

    camera_inputs_.left_mouse   = (button == GLFW_MOUSE_BUTTON_LEFT) && (action == GLFW_PRESS);
    camera_inputs_.middle_mouse = (button == GLFW_MOUSE_BUTTON_MIDDLE) && (action == GLFW_PRESS);
    camera_inputs_.right_mouse  = (button == GLFW_MOUSE_BUTTON_RIGHT) && (action == GLFW_PRESS);
}

//--------------------------------------------------------------------------------------------------
// Window callback when the mouse wheel is modified
// - Handling ImGui and a default camera
//

void AppBase::onMouseWheel(int delta)
{
    if (ImGui::GetCurrentContext() != nullptr && ImGui::GetIO().WantCaptureMouse) {
        return;
    }

    //CameraManip.wheel(delta > 0 ? 1 : -1, m_inputs);
    camera_->Wheel(delta > 0 ? 1 : -1, camera_inputs_);
}

//--------------------------------------------------------------------------------------------------
// Fit the camera to the Bounding box
//


//--------------------------------------------------------------------------------------------------
// Initialization of the GUI
// - Need to be call after the device creation
//

void AppBase::initGUI(uint32_t subpassID)
{
    assert(m_renderPass && "Render Pass must be set");

    // UI
    ImGui::CreateContext();
    ImGuiIO& io    = ImGui::GetIO();
    io.IniFilename = nullptr;  // Avoiding the INI file
    io.LogFilename = nullptr;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    ImGuiH::setStyle();
    ImGuiH::setFonts();

    ImGui::InitVK(m_device, m_physicalDevice, m_queue, m_graphicsQueueIndex, m_renderPass,
                  subpassID);
}

void AppBase::fitCamera(const glm::vec3& boxMin, const glm::vec3& boxMax, bool instantFit)
{
    //CameraManip.fit(boxMin, boxMax, instantFit, false, m_size.width / cast_f32(m_size.height));
    camera_->Fit(boxMin, boxMax, /*animated =*/false);
}

// Return true if the window is minimized

bool AppBase::isMinimized(bool doSleeping)
{
    int w, h;
    glfwGetWindowSize(m_window, &w, &h);
    bool minimized(w == 0 || h == 0);
    if (minimized && doSleeping) {
#ifdef _WIN32
        Sleep(50);
#else
        usleep(50);
#endif
    }
    return minimized;
}

// GLFW Callback setup

void AppBase::setupGlfwCallbacks(GLFWwindow* window)
{
    m_window = window;
    glfwSetWindowUserPointer(window, this);
    glfwSetKeyCallback(window, &key_cb);
    glfwSetCharCallback(window, &char_cb);
    glfwSetCursorPosCallback(window, &cursorpos_cb);
    glfwSetMouseButtonCallback(window, &mousebutton_cb);
    glfwSetScrollCallback(window, &scroll_cb);
    glfwSetFramebufferSizeCallback(window, &framebuffersize_cb);
    glfwSetDropCallback(window, &drop_cb);
}

uint32_t AppBase::getMemoryType(uint32_t typeBits, const vk::MemoryPropertyFlags& properties) const
{
    auto deviceMemoryProperties = m_physicalDevice.getMemoryProperties();
    for (uint32_t i = 0; i < deviceMemoryProperties.memoryTypeCount; i++) {
        if (((typeBits & (1 << i)) > 0)
            && (deviceMemoryProperties.memoryTypes[i].propertyFlags & properties) == properties) {
            return i;
        }
    }
    std::string err = "Unable to find memory type " + vk::to_string(properties);
    LOG(FATAL) << err;
    return ~0u;
}

// Showing help

void AppBase::uiDisplayHelp()
{
    if (m_showHelp) {
        ImGui::BeginChild("Help", ImVec2(370, 120), true);
        //ImGui::Text("%s", CameraManip.getHelp().c_str());
        ImGui::EndChild();
    }
}

}  // namespace vkpbr