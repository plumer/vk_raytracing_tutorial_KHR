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
#pragma once

#include <vulkan/vulkan.hpp>

//#include "imgui.h"
//#include "imgui_camera_widget.h"
#include "camera.h"
//#include "imgui_helper.h"
//#include "imgui_impl_vk.h"
//#include "nvh/cameramanipulator.hpp"
#include "vk_swapchain.h"

#ifdef LINUX
#include <unistd.h>
#endif

#ifdef _WIN32
#define GLFW_EXPOSE_NATIVE_WIN32
#endif
#include "GLFW/glfw3.h"
#include "GLFW/glfw3native.h"

#include <cmath>
#include <set>
#include <vector>
#include <glm/glm.hpp>

namespace vkpbr {

//--------------------------------------------------------------------------------------------------
/**

# class nvvk::AppBase

The framework comes with a few `App???` classes, these can serve as base classes for various
samples. They might differ a bit in setup and functionality, but in principle aid the setup of
context and window, as well as some common event processing.

The nvvk::AppBase serves as the base class for many ray tracing examples and makes use of the Vulkan
C++ API (`vulkan.hpp`). It does the basics for Vulkan, by holding a reference to the instance and
device, but also comes with optional default setups for the render passes and the swapchain.

## Usage

An example will derive from this class:

~~~~ C++
class MyExample : public AppBase
{
};
~~~~

## Setup

In the `main()` of an application,  call `setup()` which is taking a Vulkan instance, device,
physical device, and a queue family index.  Setup copies the given Vulkan handles into AppBase, and
query the 0th VkQueue of the specified family, which must support graphics operations and drawing to
the surface passed to createSurface. Furthermore, it is creating a VkCommandPool plus it will
initialize all Vulkan extensions for the C++ API (vulkan.hpp). See:
[VULKAN_HPP_DEFAULT_DISPATCHER](https://github.com/KhronosGroup/Vulkan-Hpp#vulkan_hpp_default_dispatcher)

Prior to calling setup, if you are using the `nvvk::Context` class to create and initialize Vulkan
instances, you may want to create a VkSurfaceKHR from the window (glfw for example) and call
`setGCTQueueWithPresent()`. This will make sure the m_queueGCT queue of nvvk::Context can draw to
the surface, and m_queueGCT.familyIndex will meet the requirements of setup().

Creating the swapchain for displaying. Arguments are
width and height, color and depth format, and vsync on/off. Defaults will create the best format for
the surface.


Creating framebuffers has a dependency on the renderPass and depth buffer. All those are virtual and
can be overridden in a sample, but default implementation exist.

- createDepthBuffer: creates a 2D depth/stencil image
- createRenderPass : creates a color/depth pass and clear both buffers.

Here is the dependency order:

~~~~C++
  example.createDepthBuffer();
  example.createRenderPass();
  example.createFrameBuffers();
~~~~


The nvvk::Swapchain will create n images, typically 3. With this information, AppBase is also
creating 3 VkFence, 3 VkCommandBuffer and 3 VkFrameBuffer.

### Frame Buffers

The created frame buffers are “display?frame buffers,  made to be presented on screen. The frame
buffers will be created using one of the images from swapchain, and a depth buffer. There is only
one depth buffer because that resource is not used simultaneously. For example, when we clear the
depth buffer, it is not done immediately, but done through a command buffer, which will be executed
later.


**Note**: the imageView(s) are part of the swapchain.

### Command Buffers

AppBase works with 3 “frame command buffers? Each frame is filling a command buffer and gets
submitted, one after the other. This is a design choice that can be debated, but makes it simple. I
think it is still possible to submit other command buffers in a frame, but those command buffers
will have to be submitted before the “frame?one. The “frame? command buffer when submitted with
submitFrame, will use the current fence.

### Fences

There are as many fences as there are images in the swapchain. At the beginning of a frame, we call
prepareFrame(). This is calling the acquire() from nvvk::SwapChain and wait until the image is
available. The very first time, the fence will not stop, but later it will wait until the submit is
completed on the GPU.



## ImGui

If the application is using Dear ImGui, there are convenient functions for initializing it and
setting the callbacks (glfw). The first one to call is `initGUI(0)`, where the argument is the
subpass it will be using. Default is 0, but if the application creates a renderpass with
multi-sampling and resolves in the second subpass, this makes it possible.

## Glfw Callbacks

Call `setupGlfwCallbacks(window)` to have all the window callback: key, mouse, window resizing.
By default AppBase will handle resizing of the window and will recreate the images and framebuffers.
If a sample needs to be aware of the resize, it can implement `onResize(width, height)`.

To handle the callbacks in Imgui, call `ImGui_ImplGlfw_InitForVulkan(window, true)`, where true
will handle the default ImGui callback .

**Note**: All the methods are virtual and can be overloaded if they are not doing the typical setup.

~~~~ C++
MyExample example;

const vk::SurfaceKHR surface = example.getVkSurface(vkctx.m_instance, window);
vkctx.setGCTQueueWithPresent(surface);

example.setup(vkctx.m_instance, vkctx.m_device, vkctx.m_physicalDevice,
vkctx.m_queueGCT.familyIndex); example.createSurface(surface, SAMPLE_SIZE_WIDTH,
SAMPLE_SIZE_HEIGHT); example.createDepthBuffer(); example.createFrameBuffers();
example.createRenderPass();
example.initGUI(0);
example.setupGlfwCallbacks(window);

ImGui_ImplGlfw_InitForVulkan(window, true);
~~~~

## Drawing loop

The drawing loop in the main() is the typicall loop you will find in glfw examples. Note that
AppBase has a convenient function to tell if the window is minimize, therefore not doing any
work and contain a sleep(), so the CPU is not going crazy.


~~~~ C++
// Window system loop
while(!glfwWindowShouldClose(window))
{
  glfwPollEvents();
  if(example.isMinimized())
    continue;

  example.display();  // infinitely drawing
}
~~~~

## Display

A typical display() function will need the following:

* Acquiring the next image: `prepareFrame()`
* Get the command buffer for the frame. There are n command buffers equal to the number of in-flight
frames.
* Clearing values
* Start rendering pass
* Drawing
* End rendering
* Submitting frame to display

~~~~ C++
void MyExample::display()
{
  // Acquire
  prepareFrame();

  // Command buffer for current frame
  const vk::CommandBuffer& cmdBuff = m_commandBuffers[getCurFrame()];
  cmdBuff.begin({vk::CommandBufferUsageFlagBits::eOneTimeSubmit});

  // Clearing values
  vk::ClearValue clearValues[2];
  clearValues[0].setColor(std::array<float, 4>({0.1f, 0.1f, 0.4f, 0.f}));
  clearValues[1].setDepthStencil({1.0f, 0});

  // Begin rendering
  vk::RenderPassBeginInfo renderPassBeginInfo{m_renderPass, m_framebuffers[getCurFrame()], {{},
m_size}, 2, clearValues}; cmdBuff.beginRenderPass(renderPassBeginInfo,
vk::SubpassContents::eInline);

  // .. draw scene ...

  // Draw UI
  ImGui::RenderDrawDataVK(cmdBuff, ImGui::GetDrawData());

  // End rendering
  cmdBuff.endRenderPass();

  // End of the frame and present the one which is ready
  cmdBuff.end();
  submitFrame();
}
~~~~~

## Closing

Finally, all resources can be destroyed by calling `destroy()` at the end of main().

~~~~ C++
  example.destroy();
~~~~

*/

class AppBase
{
  public:
    AppBase()          = default;
    virtual ~AppBase() = default;

    virtual void onResize(int /*w*/, int /*h*/){};  // To implement when the size of the window
                                                    // change
    void SetCamera(vkpbr::CameraNavigator* camera) { camera_ = camera; }


    //--------------------------------------------------------------------------------------------------
    // Setup the low level Vulkan for various operations
    //
    virtual void setup(const vk::Instance& instance, const vk::Device& device,
                       const vk::PhysicalDevice& physicalDevice, uint32_t graphicsQueueIndex);

    //--------------------------------------------------------------------------------------------------
    // To call on exit
    //
    virtual void destroy();


    //--------------------------------------------------------------------------------------------------
    // Return the surface "screen" for the display
    //
    VkSurfaceKHR getVkSurface(const vk::Instance& instance, GLFWwindow* window);


    //--------------------------------------------------------------------------------------------------
    // Creating the surface for rendering
    //
    void createSwapchain(const vk::SurfaceKHR& surface, uint32_t width, uint32_t height,
                         vk::Format colorFormat = vk::Format::eB8G8R8A8Unorm,
                         vk::Format depthFormat = vk::Format::eD32SfloatS8Uint, bool vsync = false);

    //--------------------------------------------------------------------------------------------------
    // Create the framebuffers in which the image will be rendered
    // - Swapchain need to be created before calling this
    //
    virtual void createFrameBuffers();

    //--------------------------------------------------------------------------------------------------
    // Creating a default render pass, very simple one.
    // Other examples will mostly override this one.
    //
    virtual void createRenderPass();

    //--------------------------------------------------------------------------------------------------
    // Creating an image to be used as depth buffer
    //
    virtual void createDepthBuffer();

    //--------------------------------------------------------------------------------------------------
    // Convenient function to call before rendering
    //
    void prepareFrame()
    {
        // Resize protection - should be cached by the glFW callback
        int w, h;
        glfwGetFramebufferSize(m_window, &w, &h);
        if (w != (int)m_size.width || h != (int)m_size.height) {
            onFramebufferSize(w, h);
        }

        // Acquire the next image from the swap chain
        if (!m_swapChain.acquire()) {
            assert(!"This shouldn't happen");
        }

        // Use a fence to wait until the command buffer has finished execution before using it again
        uint32_t imageIndex = m_swapChain.getActiveImageIndex();
        while (m_device.waitForFences(m_waitFences[imageIndex], VK_TRUE, 10000)
               == vk::Result::eTimeout) {}
    }

    //--------------------------------------------------------------------------------------------------
    // Convenient function to call for submitting the rendering command
    //
    void submitFrame();


    //--------------------------------------------------------------------------------------------------
    // When the pipeline is set for using dynamic, this becomes useful
    //
    void setViewport(const vk::CommandBuffer& cmdBuf);

    //--------------------------------------------------------------------------------------------------
    // Window callback when the it is resized
    // - Destroy allocated frames, then rebuild them with the new size
    // - Call onResize() of the derived class
    //
    virtual void onFramebufferSize(int w, int h);


    //--------------------------------------------------------------------------------------------------
    // Window callback when the mouse move
    // - Handling ImGui and a default camera
    //
    virtual void onMouseMotion(int x, int y);

    //--------------------------------------------------------------------------------------------------
    // Window callback when a special key gets hit
    // - Handling ImGui and a default camera
    //
    virtual void onKeyboard(int key, int scancode, int action, int mods);

    //--------------------------------------------------------------------------------------------------
    // Window callback when a key gets hit
    //
    virtual void onKeyboardChar(unsigned char key);

    //--------------------------------------------------------------------------------------------------
    // Window callback when the mouse button is pressed
    // - Handling ImGui and a default camera
    //
    virtual void onMouseButton(int button, int action, int mods);

    //--------------------------------------------------------------------------------------------------
    // Window callback when the mouse wheel is modified
    // - Handling ImGui and a default camera
    //
    virtual void onMouseWheel(int delta);

    virtual void onFileDrop(const char* filename) {}

    //--------------------------------------------------------------------------------------------------
    // Initialization of the GUI
    // - Need to be call after the device creation
    //
    void initGUI(uint32_t subpassID = 0);

    //--------------------------------------------------------------------------------------------------
    // Fit the camera to the Bounding box
    //
    void fitCamera(const glm::vec3& boxMin, const glm::vec3& boxMax,
                   bool instantFit = true);

    // Return true if the window is minimized
    bool isMinimized(bool doSleeping = true);

    void setTitle(const std::string& title) { glfwSetWindowTitle(m_window, title.c_str()); }

    // GLFW Callback setup
    void        setupGlfwCallbacks(GLFWwindow* window);
    static void framebuffersize_cb(GLFWwindow* window, int w, int h)
    {
        auto app = reinterpret_cast<AppBase*>(glfwGetWindowUserPointer(window));
        app->onFramebufferSize(w, h);
    }
    static void mousebutton_cb(GLFWwindow* window, int button, int action, int mods)
    {
        auto app = reinterpret_cast<AppBase*>(glfwGetWindowUserPointer(window));
        app->onMouseButton(button, action, mods);
    }
    static void cursorpos_cb(GLFWwindow* window, double x, double y)
    {
        auto app = reinterpret_cast<AppBase*>(glfwGetWindowUserPointer(window));
        app->onMouseMotion(static_cast<int>(x), static_cast<int>(y));
    }
    static void scroll_cb(GLFWwindow* window, double x, double y)
    {
        auto app = reinterpret_cast<AppBase*>(glfwGetWindowUserPointer(window));
        app->onMouseWheel(static_cast<int>(y));
    }
    static void key_cb(GLFWwindow* window, int key, int scancode, int action, int mods)
    {
        auto app = reinterpret_cast<AppBase*>(glfwGetWindowUserPointer(window));
        app->onKeyboard(key, scancode, action, mods);
    }
    static void char_cb(GLFWwindow* window, unsigned int key)
    {
        auto app = reinterpret_cast<AppBase*>(glfwGetWindowUserPointer(window));
        app->onKeyboardChar(key);
    }
    static void drop_cb(GLFWwindow* window, int count, const char** paths)
    {
        auto app = reinterpret_cast<AppBase*>(glfwGetWindowUserPointer(window));
        int  i;
        for (i = 0; i < count; i++)
            app->onFileDrop(paths[i]);
    }
    // GLFW Callback end

    // Set if Nvlink will be used
    void useNvlink(bool useNvlink) { m_useNvlink = useNvlink; }

    //--------------------------------------------------------------------------------------------------
    // Getters
    vk::Instance       getInstance() { return m_instance; }
    vk::Device         getDevice() { return m_device; }
    vk::PhysicalDevice getPhysicalDevice() { return m_physicalDevice; }
    vk::Queue          getQueue() { return m_queue; }
    uint32_t           getQueueFamily() { return m_graphicsQueueIndex; }
    vk::CommandPool    getCommandPool() { return m_cmdPool; }
    vk::RenderPass     getRenderPass() { return m_renderPass; }
    vk::Extent2D       getSize() { return m_size; }
    vk::PipelineCache  getPipelineCache() { return m_pipelineCache; }
    vk::SurfaceKHR     getSurface() { return m_surface; }

    const std::vector<vk::Framebuffer>&   getFramebuffers() { return m_framebuffers; }
    const std::vector<vk::CommandBuffer>& getCommandBuffers() { return m_commandBuffers; }
    uint32_t   getCurFrame() const { return m_swapChain.getActiveImageIndex(); }
    vk::Format getColorFormat() const { return m_colorFormat; }
    vk::Format getDepthFormat() const { return m_depthFormat; }
    bool       showGui() { return m_show_gui; }

  protected:
    uint32_t getMemoryType(uint32_t typeBits, const vk::MemoryPropertyFlags& properties) const;

    // Showing help
    void uiDisplayHelp();

    //--------------------------------------------------------------------------------------------------

    // Vulkan low level
    vk::Instance       m_instance;
    vk::Device         m_device;
    vk::SurfaceKHR     m_surface;
    vk::PhysicalDevice m_physicalDevice;
    vk::Queue          m_queue;
    uint32_t           m_graphicsQueueIndex{VK_QUEUE_FAMILY_IGNORED};
    vk::CommandPool    m_cmdPool;

    // Drawing/Surface
    SwapChain                m_swapChain;
    std::vector<vk::Framebuffer>   m_framebuffers;  // All framebuffers, correspond to the Swapchain
    std::vector<vk::CommandBuffer> m_commandBuffers;  // Command buffer per nb element in Swapchain
    std::vector<vk::Fence>         m_waitFences;      // Fences per nb element in Swapchain
    vk::Image                      m_depthImage;      // Depth/Stencil
    vk::DeviceMemory               m_depthMemory;     // Depth/Stencil
    vk::ImageView                  m_depthView;       // Depth/Stencil
    vk::RenderPass                 m_renderPass;      // Base render pass
    vk::Extent2D                   m_size{0, 0};      // Size of the window
    vk::PipelineCache              m_pipelineCache;   // Cache for pipeline/shaders
    bool                           m_vsync{false};    // Swapchain with vsync
    bool                           m_useNvlink{false};  // NVLINK usage
    GLFWwindow*                    m_window{nullptr};   // GLFW Window

    // Surface buffer formats
    vk::Format m_colorFormat{vk::Format::eB8G8R8A8Unorm};
    vk::Format m_depthFormat{vk::Format::eUndefined};

    // Camera manipulators
    //nvh::CameraManipulator::Inputs m_inputs;  // Mouse button pressed
    CameraNavigator::Inputs camera_inputs_;
    vkpbr::CameraNavigator*        camera_ = nullptr;
    std::set<int>                  m_keys;    // Keyboard pressed

    // Other
    bool m_showHelp{false};  // Show help, pressing
    bool m_show_gui{true};
};  // namespace nvvk


}  // namespace nvvk
