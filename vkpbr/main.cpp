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

// ImGui - standalone example application for Glfw + Vulkan, using programmable
// pipeline If you are new to ImGui, see examples/README.txt and documentation
// at the top of imgui.cpp.

#include <array>
#include <vulkan/vulkan.hpp>
VULKAN_HPP_DEFAULT_DISPATCH_LOADER_DYNAMIC_STORAGE

#include "imgui.h"
#include "imgui_impl_glfw.h"

#include "hello_vulkan.h"
//#include "imgui_camera_widget.h"
#include "imgui_helper.h"
#include "imgui_impl_vk.h"
//#include "nvh/cameramanipulator.hpp"
#include "io.h"
#include "nvpsystem.hpp"
#include "vk_appbase.h"
#include "vk_utils.h"


//////////////////////////////////////////////////////////////////////////
#define UNUSED(x) (void)(x)
//////////////////////////////////////////////////////////////////////////

// Default search path for shaders
std::vector<std::string> defaultSearchPaths;

// GLFW Callback functions
static void onErrorCallback(int error, const char* description)
{
    fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

// Extra UI
void renderUI(HelloVulkan& helloVk)
{
    // ImGuiH::CameraWidget();
    if (ImGui::CollapsingHeader("Light")) {
        ImGui::RadioButton("Point", &helloVk.m_pushConstant.lightType, 0);
        ImGui::SameLine();
        ImGui::RadioButton("Infinite", &helloVk.m_pushConstant.lightType, 1);

        ImGui::SliderFloat3("Position", &helloVk.m_pushConstant.lightPosition.x, -20.f, 20.f);
        ImGui::SliderFloat("Intensity", &helloVk.m_pushConstant.lightIntensity, 0.f, 150.f);
    }
}

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
static int const SAMPLE_WIDTH  = 1280;
static int const SAMPLE_HEIGHT = 720;

//--------------------------------------------------------------------------------------------------
// Application Entry
//
int main(int argc, char** argv)
{
    UNUSED(argc);

    // Setup GLFW window
    glfwSetErrorCallback(onErrorCallback);
    if (!glfwInit()) {
        return 1;
    }
    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    GLFWwindow* window =
        glfwCreateWindow(SAMPLE_WIDTH, SAMPLE_HEIGHT, PROJECT_NAME, nullptr, nullptr);

    vkpbr::CameraNavigator camera;
    camera.SetWindowSize(SAMPLE_WIDTH, SAMPLE_HEIGHT);
    camera.SetLookAt(glm::vec3(5, 4, -4), {0, 1, 0}, {0, 1, 0});

    // Setup Vulkan
    if (!glfwVulkanSupported()) {
        printf("GLFW: Vulkan Not Supported\n");
        return 1;
    }

    // setup some basic things for the sample, logging file for example
    NVPSystem system(argv[0], PROJECT_NAME);

    // Search path for shaders and other media
    defaultSearchPaths = {
        PROJECT_ABSDIRECTORY,
        PROJECT_ABSDIRECTORY "..",
        NVPSystem::exePath(),
        NVPSystem::exePath() + "..",
        NVPSystem::exePath() + std::string(PROJECT_NAME),
    };

    // Requesting Vulkan extensions and layers
    vkpbr::ContextCreateInfo contextInfo(true);
    contextInfo.SetVersion(1, 2);
    contextInfo.AddInstanceLayer("VK_LAYER_LUNARG_monitor", true);
    contextInfo.AddInstanceExtension(VK_EXT_DEBUG_UTILS_EXTENSION_NAME, true);
    contextInfo.AddInstanceExtension(VK_KHR_SURFACE_EXTENSION_NAME);
#ifdef WIN32
    contextInfo.AddInstanceExtension(VK_KHR_WIN32_SURFACE_EXTENSION_NAME);
#else
    contextInfo.AddInstanceExtension(VK_KHR_XLIB_SURFACE_EXTENSION_NAME);
    contextInfo.AddInstanceExtension(VK_KHR_XCB_SURFACE_EXTENSION_NAME);
#endif
    contextInfo.AddInstanceExtension(VK_KHR_GET_PHYSICAL_DEVICE_PROPERTIES_2_EXTENSION_NAME);
    contextInfo.AddDeviceExtension(VK_KHR_SWAPCHAIN_EXTENSION_NAME);
    contextInfo.AddDeviceExtension(VK_KHR_DEDICATED_ALLOCATION_EXTENSION_NAME);
    contextInfo.AddDeviceExtension(VK_KHR_GET_MEMORY_REQUIREMENTS_2_EXTENSION_NAME);
    // #VKRay: Activate the ray tracing extension
    vk::PhysicalDeviceAccelerationStructureFeaturesKHR accelFeature;
    contextInfo.AddDeviceExtension(VK_KHR_ACCELERATION_STRUCTURE_EXTENSION_NAME, false,
                                   &accelFeature);
    vk::PhysicalDeviceRayTracingPipelineFeaturesKHR rtPipelineFeature;
    contextInfo.AddDeviceExtension(VK_KHR_RAY_TRACING_PIPELINE_EXTENSION_NAME, false,
                                   &rtPipelineFeature);
    contextInfo.AddDeviceExtension(VK_KHR_MAINTENANCE3_EXTENSION_NAME);
    contextInfo.AddDeviceExtension(VK_KHR_PIPELINE_LIBRARY_EXTENSION_NAME);
    contextInfo.AddDeviceExtension(VK_KHR_DEFERRED_HOST_OPERATIONS_EXTENSION_NAME);
    contextInfo.AddDeviceExtension(VK_KHR_BUFFER_DEVICE_ADDRESS_EXTENSION_NAME);


    // Creating Vulkan base application
    vkpbr::Context vkctx{};
    vkctx.InitInstance(contextInfo);
    // Find all compatible devices
    auto compatibleDevices = vkctx.GetCompatibleDevices(contextInfo);
    assert(!compatibleDevices.empty());
    // Use a compatible device
    vkctx.InitDevice(compatibleDevices[0], contextInfo);


    // Create example
    HelloVulkan helloVk;

    // Window need to be opened to get the surface on which to draw
    const vk::SurfaceKHR surface = helloVk.getVkSurface(vkctx.m_instance, window);
    vkctx.SetGCTQueueWithPresent(surface);

    helloVk.setup(vkctx.m_instance, vkctx.m_device, vkctx.m_physicalDevice,
                  vkctx.m_queueGCT.familyIndex);
    helloVk.SetCamera(&camera);
    helloVk.createSwapchain(surface, SAMPLE_WIDTH, SAMPLE_HEIGHT);
    helloVk.createDepthBuffer();
    helloVk.createRenderPass();
    helloVk.createFrameBuffers();

    // Setup Imgui
    helloVk.initGUI(0);  // Using sub-pass 0

    // Creation of the example
    helloVk.loadModel(io::FindFile("media/scenes/Medieval_building.obj", defaultSearchPaths));
    helloVk.loadModel(io::FindFile("media/scenes/plane.obj", defaultSearchPaths));


    helloVk.createOffscreenRender();
    helloVk.createDescriptorSetLayout();
    helloVk.createGraphicsPipeline();
    helloVk.createUniformBuffer();
    helloVk.createSceneDescriptionBuffer();
    helloVk.updateDescriptorSet();

    // #VKRay
    helloVk.initRayTracing();
    helloVk.createBottomLevelAS();
    helloVk.createTopLevelAS();
    helloVk.createRtDescriptorSet();
    helloVk.createRtPipeline();
    helloVk.createRtShaderBindingTable();

    helloVk.createPostDescriptor();
    helloVk.createPostPipeline();
    helloVk.updatePostDescriptorSet();


    glm::vec4 clearColor   = glm::vec4(1, 1, 1, 1.00f);
    bool      useRaytracer = true;


    helloVk.setupGlfwCallbacks(window);
    ImGui_ImplGlfw_InitForVulkan(window, true);

    // Main loop
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        if (helloVk.isMinimized())
            continue;

        // Start the Dear ImGui frame
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();


        // Show UI window.
        if (helloVk.showGui()) {
            ImGuiH::Panel::Begin();
            ImGui::ColorEdit3("Clear color", reinterpret_cast<float*>(&clearColor));
            ImGui::Checkbox("Ray Tracer mode", &useRaytracer);  // Switch between raster and ray
                                                                // tracing

            renderUI(helloVk);
            ImGui::Text("Application average %.3f ms/frame (%.1f FPS)",
                        1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

            ImGuiH::Control::Info("", "", "(F10) Toggle Pane", ImGuiH::Control::Flags::Disabled);
            ImGuiH::Panel::End();
        }

        // Start rendering the scene
        helloVk.prepareFrame();

        // Start command buffer of this frame
        auto                     curFrame = helloVk.CurrentFrameIndex();
        const vk::CommandBuffer& cmdBuf   = helloVk.getCommandBuffers()[curFrame];

        cmdBuf.begin({vk::CommandBufferUsageFlagBits::eOneTimeSubmit});

        // Updating camera buffer
        helloVk.updateUniformBuffer(cmdBuf);

        // Clearing screen
        vk::ClearValue clearValues[2];
        clearValues[0].setColor(
            std::array<float, 4>({clearColor[0], clearColor[1], clearColor[2], clearColor[3]}));
        clearValues[1].setDepthStencil({1.0f, 0});

        // Offscreen render pass
        {
            vk::RenderPassBeginInfo offscreenRenderPassBeginInfo;
            offscreenRenderPassBeginInfo.setClearValueCount(2);
            offscreenRenderPassBeginInfo.setPClearValues(clearValues);
            offscreenRenderPassBeginInfo.setRenderPass(helloVk.m_offscreenRenderPass);
            offscreenRenderPassBeginInfo.setFramebuffer(helloVk.m_offscreenFramebuffer);
            offscreenRenderPassBeginInfo.setRenderArea({{}, helloVk.getSize()});

            // Rendering Scene
            if (useRaytracer) {
                helloVk.raytrace(cmdBuf, clearColor);
            } else {
                cmdBuf.beginRenderPass(offscreenRenderPassBeginInfo, vk::SubpassContents::eInline);
                helloVk.rasterize(cmdBuf);
                cmdBuf.endRenderPass();
            }
        }

        // 2nd rendering pass: tone mapper, UI
        {
            vk::RenderPassBeginInfo postRenderPassBeginInfo;
            postRenderPassBeginInfo.setClearValueCount(2);
            postRenderPassBeginInfo.setPClearValues(clearValues);
            postRenderPassBeginInfo.setRenderPass(helloVk.RenderPass());
            postRenderPassBeginInfo.setFramebuffer(helloVk.getFramebuffers()[curFrame]);
            postRenderPassBeginInfo.setRenderArea({{}, helloVk.getSize()});

            cmdBuf.beginRenderPass(postRenderPassBeginInfo, vk::SubpassContents::eInline);
            // Rendering tonemapper
            helloVk.drawPost(cmdBuf);
            // Rendering UI
            ImGui::Render();
            ImGui::RenderDrawDataVK(cmdBuf, ImGui::GetDrawData());
            cmdBuf.endRenderPass();
        }

        // Submit for display
        cmdBuf.end();
        helloVk.submitFrame();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    // Cleanup
    helloVk.Device().waitIdle();
    helloVk.destroyResources();
    helloVk.destroy();

    vkctx.DeInit();

    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}
