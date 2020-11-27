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

<<<<<<< HEAD
#include <tiny_gltf.h>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_vk.h"

#include "nvh/gltfscene.hpp"

#include "hello_vulkan.h"
#include "io.h"
#include "logging.h"
#include <glm/gtx/string_cast.hpp>
#include <thread>
=======
#include "imgui.h"
#include "imgui_impl_glfw.h"

#include "hello_vulkan.h"
#include "imgui_camera_widget.h"
#include "nvh/cameramanipulator.hpp"
#include "nvh/fileoperations.hpp"
#include "nvpsystem.hpp"
#include "nvvk/appbase_vkpp.hpp"
#include "nvvk/commands_vk.hpp"
#include "nvvk/context_vk.hpp"

>>>>>>> b7ba7d4fdc4128cefb7ab1d6097cc6d21c37d817

//////////////////////////////////////////////////////////////////////////
#define UNUSED(x) (void)(x)
//////////////////////////////////////////////////////////////////////////

// Default search path for shaders
std::vector<std::string> defaultSearchPaths;

// GLFW Callback functions
static void onErrorCallback(int error, const char* description)
{
<<<<<<< HEAD
    fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

// Extra UI
void RenderUI(HelloVulkan& helloVk);
=======
  fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

// Extra UI
void renderUI(HelloVulkan& helloVk)
{
  ImGuiH::CameraWidget();
  if(ImGui::CollapsingHeader("Light"))
  {
    ImGui::RadioButton("Point", &helloVk.m_pushConstant.lightType, 0);
    ImGui::SameLine();
    ImGui::RadioButton("Infinite", &helloVk.m_pushConstant.lightType, 1);

    ImGui::SliderFloat3("Position", &helloVk.m_pushConstant.lightPosition.x, -20.f, 20.f);
    ImGui::SliderFloat("Intensity", &helloVk.m_pushConstant.lightIntensity, 0.f, 150.f);
  }
}
>>>>>>> b7ba7d4fdc4128cefb7ab1d6097cc6d21c37d817

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
static int const SAMPLE_WIDTH  = 1280;
<<<<<<< HEAD
static int const SAMPLE_HEIGHT = 800;
=======
static int const SAMPLE_HEIGHT = 720;
>>>>>>> b7ba7d4fdc4128cefb7ab1d6097cc6d21c37d817

//--------------------------------------------------------------------------------------------------
// Application Entry
//
int main(int argc, char** argv)
{
<<<<<<< HEAD
    UNUSED(argc);

    std::ios::sync_with_stdio(true);

    // Setup GLFW window
    glfwSetErrorCallback(onErrorCallback);
    if (!glfwInit()) {
        return 1;
    }
    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    GLFWwindow* window = glfwCreateWindow(SAMPLE_WIDTH, SAMPLE_HEIGHT,
                                          "NVIDIA Vulkan Raytracing Tutorial", nullptr, nullptr);

    // Setup camera
    auto camera_navigator = std::make_shared<vkpbr::CameraNavigator>();
    camera_navigator->SetWindowSize(SAMPLE_WIDTH, SAMPLE_HEIGHT);
    camera_navigator->SetLookAt(glm::vec3(0, 0, 15), glm::vec3(0, 0, 0), glm::vec3(0, 1, 0));

    // Setup Vulkan
    if (!glfwVulkanSupported()) {
        printf("GLFW: Vulkan Not Supported\n");
        return 1;
    }

    // Search path for shaders and other media
    // --------------------------------------------------------------------------------------------
    std::string executable_path;
    {
        std::string logfile = std::string("log_") + std::string(PROJECT_NAME) + std::string(".txt");

        std::string exe = argv[0];
        std::replace(exe.begin(), exe.end(), '\\', '/');

        size_t last = exe.rfind('/');
        if (last != std::string::npos) {
            executable_path = exe.substr(0, last) + std::string("/");
        }
    }
    defaultSearchPaths = {
        PROJECT_ABSDIRECTORY,
        PROJECT_ABSDIRECTORY "../",
        executable_path + std::string(PROJECT_RELDIRECTORY),
        executable_path + std::string(PROJECT_RELDIRECTORY) + std::string("../"),
    };

    // Enabling the extension feature
    vk::PhysicalDeviceRayTracingFeaturesKHR raytracingFeature;

    // Creates the context: instance, device and queues.
    // --------------------------------------------------------------------------------------------
    vkpbr::ContextCreateInfo context_ci(/* use_validation=*/true);
    context_ci.SetVersion(1, 2);
    context_ci.AddInstanceLayer("VK_LAYER_LUNARG_monitor", /*optional = */ true);
    context_ci.AddInstanceExtension(
        {VK_KHR_SURFACE_EXTENSION_NAME, VK_KHR_GET_PHYSICAL_DEVICE_PROPERTIES_2_EXTENSION_NAME});
#ifdef WIN32
    context_ci.AddInstanceExtension(VK_KHR_WIN32_SURFACE_EXTENSION_NAME);
#else
    context_ci.AddInstanceExtension(
        {VK_KHR_XLIB_SURFACE_EXTENSION_NAME, VK_KHR_XCB_SURFACE_EXTENSION_NAME});
#endif
    context_ci.AddDeviceExtension(
        {VK_KHR_SWAPCHAIN_EXTENSION_NAME, VK_KHR_DEDICATED_ALLOCATION_EXTENSION_NAME,
         VK_KHR_GET_MEMORY_REQUIREMENTS_2_EXTENSION_NAME, VK_EXT_DESCRIPTOR_INDEXING_EXTENSION_NAME,
         VK_EXT_SCALAR_BLOCK_LAYOUT_EXTENSION_NAME, VK_KHR_MAINTENANCE3_EXTENSION_NAME,
         VK_KHR_PIPELINE_LIBRARY_EXTENSION_NAME, VK_KHR_DEFERRED_HOST_OPERATIONS_EXTENSION_NAME,
         VK_KHR_BUFFER_DEVICE_ADDRESS_EXTENSION_NAME, VK_KHR_SHADER_CLOCK_EXTENSION_NAME});
    context_ci.AddDeviceExtension(VK_KHR_RAY_TRACING_EXTENSION_NAME, /*optional=*/false,
                                  &raytracingFeature);
    vkpbr::Context vk_context;
    vk_context.InitInstance(context_ci);
    auto compatible_gpus = vk_context.GetCompatibleDevices(context_ci);
    CHECK(!compatible_gpus.empty());
    CHECK(vk_context.InitDevice(compatible_gpus[0], context_ci));


    // Create example
    // --------------------------------------------------------------------------------------------
    HelloVulkan helloVk;

    // Shares the camera with the app.
    helloVk.SetCameraNavigator(camera_navigator);
    const vk::SurfaceKHR surface = helloVk.AcquireSurface(vk_context.Instance(), window);
    vk_context.SetGCTQueueWithPresent(surface);
    helloVk.Setup(vk_context.Instance(), vk_context.Device(), vk_context.Gpu(),
                  vk_context.GetGctQueueFamilyIndex());

    helloVk.MakeSurface(surface, SAMPLE_WIDTH, SAMPLE_HEIGHT);
    helloVk.MakeDepthBuffer();
    helloVk.MakeRenderPass();
    helloVk.MakeFrameBuffers();

    // Setup Imgui
    helloVk.InitGui(0);  // Using sub-pass 0

    // Creation of the example
    //  helloVk.loadModel(io::FindFile("media/scenes/Medieval_building.obj", defaultSearchPaths));
    // helloVk.LoadModel(io::FindFile("media/scenes/plane.obj", defaultSearchPaths));
    helloVk.LoadGltfModel(io::FindFile("scenes/cornellBox.gltf", defaultSearchPaths));
    helloVk.LoadPbrtModel(io::FindFile("barcelona-pavilion/pavilion-day.pbrt", {"/home/zixun/repos/"
                                                                                "pbrt-v3-"
                                                                                "scenes/"}));
    helloVk.createSpheres();

    helloVk.createOffscreenRender();
    helloVk.BuildDescriptorSetLayout();
    helloVk.BuildGraphicsPipeline();
    helloVk.BuildUniformBuffer();
    // helloVk.BuildSceneDescriptionBuffer();
    helloVk.UpdateDescriptorSet();

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
    bool      useRaytracer = false;


    helloVk.SetupGlfwCallbacks();
    ImGui_ImplGlfw_InitForVulkan(window, true);

    // Main loop
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        if (helloVk.IsMinimized(/*sleep_if_so=*/true))
            continue;

        // Start the Dear ImGui frame
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // Updating camera buffer
        helloVk.UpdateUniformBuffer();

        // Show UI window.
        if (1 == 1) {
            ImGui::Begin("Debug");
            ImGui::ColorEdit3("Clear color", reinterpret_cast<float*>(&clearColor));
            ImGui::Checkbox("Ray Tracer mode",
                            &useRaytracer);  // Switch between raster and ray tracing

            static int item = 1;
            if (ImGui::Combo("Up Vector", &item, "X\0Y\0Z\0\0")) {
                vkpbr::CameraNavigator::Camera c = camera_navigator->GetLookAt();
                glm::vec3                      up(item == 0, item == 1, item == 2);
                camera_navigator->SetLookAt(c.eye, c.center, up);
            }
            ImGui::SliderFloat3("Light Position", &helloVk.m_pushConstant.lightPosition.x, -5.f,
                                5.f);
            ImGui::SliderFloat("Light Intensity", &helloVk.m_pushConstant.lightIntensity, 0.f,
                               100.f);
            ImGui::RadioButton("Point", &helloVk.m_pushConstant.lightType, 0);
            ImGui::SameLine();
            ImGui::RadioButton("Infinite", &helloVk.m_pushConstant.lightType, 1);
            if (ImGui::Button("Reset camera")) {
                camera_navigator->SetMode(vkpbr::CameraNavigator::Modes::kFly);
                camera_navigator->SetLookAt(glm::vec3(20, 20, 20), glm::vec3(0, 1, 0),
                                            glm::vec3(0, 1, 0), true);
                camera_navigator->SetMode(vkpbr::CameraNavigator::Modes::kExamine);
            }
            // ImGui::SliderInt("Mtl Index", &helloVk.m_rtPushConstants.selected_mtl_index, -1, 10);
            // ImGui::ColorEdit3("Mtl color", &helloVk.m_rtPushConstants.selected_mtl_color.x);
            // ImGui::SliderInt("Path length", &helloVk.m_rtPushConstants.path_length, 1, 10);

            ImGui::Text("Application average %.3f ms/frame (%.1f FPS)",
                        1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
            ImGui::Text("# of frames accumulated: %d", helloVk.m_rtPushConstants.frame);
            if (ImGui::Button("Write framebuffer to image")) {
                helloVk.WriteFramebuffer("fb.jpg");
            }
            ImGui::End();
            RenderUI(helloVk);
            ImGui::Render();
        }

        // Start rendering the scene
        helloVk.PrepareFrame();

        // Start command buffer of this frame
        auto                     curFrame = helloVk.CurrentFrameIndex();
        const vk::CommandBuffer& cmdBuff  = helloVk.cmd_buffers()[curFrame];

        cmdBuff.begin({vk::CommandBufferUsageFlagBits::eOneTimeSubmit});

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
            offscreenRenderPassBeginInfo.setRenderArea({{}, helloVk.window_size()});

            // Rendering Scene
            if (useRaytracer) {
                helloVk.raytrace(cmdBuff, clearColor);
            } else {
                cmdBuff.beginRenderPass(offscreenRenderPassBeginInfo, vk::SubpassContents::eInline);
                helloVk.rasterize(cmdBuff);
                cmdBuff.endRenderPass();
            }
        }

        // 2nd rendering pass: tone mapper, UI
        {
            vk::RenderPassBeginInfo postRenderPassBeginInfo;
            postRenderPassBeginInfo.setClearValueCount(2);
            postRenderPassBeginInfo.setPClearValues(clearValues);
            postRenderPassBeginInfo.setRenderPass(helloVk.render_pass());
            postRenderPassBeginInfo.setFramebuffer(helloVk.framebuffers()[curFrame]);
            postRenderPassBeginInfo.setRenderArea({{}, helloVk.window_size()});

            cmdBuff.beginRenderPass(postRenderPassBeginInfo, vk::SubpassContents::eInline);
            // Rendering tonemapper
            helloVk.drawPost(cmdBuff);
            // Rendering UI
            ImGui::RenderDrawDataVK(cmdBuff, ImGui::GetDrawData());
            cmdBuff.endRenderPass();
        }

        // Submit for display
        cmdBuff.end();
        helloVk.SubmitFrame();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // Cleanup
    helloVk.device().waitIdle();
    helloVk.destroyResources();
    helloVk.UnloadContext();

    vk_context.DeInit();

    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}

void RenderUI(HelloVulkan& hello_vk)
{
    static bool is_open = false;
    ImGui::Begin("PBR Texture data");
    for (const auto& [name, index] : hello_vk.pbr_scene.named_textures_) {
        ImGui::Text("Texture \'%-30s\' #%lu", name.c_str(), index);
    }
    for (int i = 0; i < hello_vk.pbr_scene.texture_descriptors_.size(); ++i) {
        const auto& tex = hello_vk.pbr_scene.texture_descriptors_[i];
        ImGui::Text("Texture #%d:", i);
        ImGui::SameLine();
        if (tex.type == vkpbr::Texture::Type::eConstant) {
            ImGui::Text("Constant, color = %s", glm::to_string(tex.constant_color).c_str());
        } else if (tex.type == vkpbr::Texture::Type::eSampledImage) {
            ImGui::Text("Sampler, image id = %lu", tex.sampler);
        } else if (tex.type == vkpbr::Texture::Type::eCheckered) {
            ImGui::Text("Checked, even id = #%lu, odd id = #%lu", tex.checker_indices[0],
                        tex.checker_indices[1]);
        } else {
            ImGui::Text("%s", tex.TypeName());
        }
    }
    ImGui::End();

    ImGui::Begin("PBR Material data");
    for (const auto& [name, index] : hello_vk.pbr_scene.named_materials_) {
        ImGui::Text("Material \'%-40s\' #%lu", name.c_str(), index);
    }
    for (int i = 0; i < hello_vk.pbr_scene.material_descriptors_.size(); ++i) {
        const auto& mtl = hello_vk.pbr_scene.material_descriptors_[i];
        ImGui::Text("Material #%d: ", i); ImGui::SameLine();
        if (mtl.type == vkpbr::Material::Type::eUber) {
            ImGui::Text("Uber, diffuse = #%lu, specular = #%lu, reflect = #%lu, transmit = #%lu",
                        mtl.diffuse_tex_id, mtl.specular_tex_id, mtl.reflective_tex_id,
                        mtl.transmissive_tex_id);
        } else if (mtl.type == vkpbr::Material::Type::eGlass) {
            ImGui::Text("Glass, specular = #%lu, reflect = #%lu, transmit = #%lu, eta = %g + %gi",
                        mtl.specular_tex_id, mtl.reflective_tex_id, mtl.transmissive_tex_id,
                        mtl.eta, mtl.eta_imaginary);
        } else {
                ImGui::Text("%s ", mtl.TypeName());  // ImGui::SameLine();
        }
    }
    ImGui::End();
}
=======
  UNUSED(argc);

  // Setup GLFW window
  glfwSetErrorCallback(onErrorCallback);
  if(!glfwInit())
  {
    return 1;
  }
  glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
  GLFWwindow* window =
      glfwCreateWindow(SAMPLE_WIDTH, SAMPLE_HEIGHT, PROJECT_NAME, nullptr, nullptr);

  // Setup camera
  CameraManip.setWindowSize(SAMPLE_WIDTH, SAMPLE_HEIGHT);
  CameraManip.setLookat(nvmath::vec3f(0, 0, 15), nvmath::vec3f(0, 0, 0), nvmath::vec3f(0, 1, 0));

  // Setup Vulkan
  if(!glfwVulkanSupported())
  {
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
      NVPSystem::exePath() + std::string(PROJECT_NAME),
  };

  // Requesting Vulkan extensions and layers
  nvvk::ContextCreateInfo contextInfo(true);
  contextInfo.setVersion(1, 2);
  contextInfo.addInstanceLayer("VK_LAYER_LUNARG_monitor", true);
  contextInfo.addInstanceExtension(VK_KHR_SURFACE_EXTENSION_NAME);
  contextInfo.addInstanceExtension(VK_EXT_DEBUG_UTILS_EXTENSION_NAME, true);
#ifdef WIN32
  contextInfo.addInstanceExtension(VK_KHR_WIN32_SURFACE_EXTENSION_NAME);
#else
  contextInfo.addInstanceExtension(VK_KHR_XLIB_SURFACE_EXTENSION_NAME);
  contextInfo.addInstanceExtension(VK_KHR_XCB_SURFACE_EXTENSION_NAME);
#endif
  contextInfo.addInstanceExtension(VK_KHR_GET_PHYSICAL_DEVICE_PROPERTIES_2_EXTENSION_NAME);
  contextInfo.addDeviceExtension(VK_KHR_SWAPCHAIN_EXTENSION_NAME);
  contextInfo.addDeviceExtension(VK_KHR_DEDICATED_ALLOCATION_EXTENSION_NAME);
  contextInfo.addDeviceExtension(VK_KHR_GET_MEMORY_REQUIREMENTS_2_EXTENSION_NAME);
  // #VKRay: Activate the ray tracing extension
  contextInfo.addDeviceExtension(VK_KHR_MAINTENANCE3_EXTENSION_NAME);
  contextInfo.addDeviceExtension(VK_KHR_PIPELINE_LIBRARY_EXTENSION_NAME);
  contextInfo.addDeviceExtension(VK_KHR_DEFERRED_HOST_OPERATIONS_EXTENSION_NAME);
  contextInfo.addDeviceExtension(VK_KHR_BUFFER_DEVICE_ADDRESS_EXTENSION_NAME);
  contextInfo.addDeviceExtension(VK_KHR_SHADER_CLOCK_EXTENSION_NAME);
  vk::PhysicalDeviceAccelerationStructureFeaturesKHR accelFeature;
  contextInfo.addDeviceExtension(VK_KHR_ACCELERATION_STRUCTURE_EXTENSION_NAME, false,
                                 &accelFeature);
  vk::PhysicalDeviceRayTracingPipelineFeaturesKHR rtPipelineFeature;
  contextInfo.addDeviceExtension(VK_KHR_RAY_TRACING_PIPELINE_EXTENSION_NAME, false,
                                 &rtPipelineFeature);


  // Creating Vulkan base application
  nvvk::Context vkctx{};
  vkctx.initInstance(contextInfo);
  // Find all compatible devices
  auto compatibleDevices = vkctx.getCompatibleDevices(contextInfo);
  assert(!compatibleDevices.empty());
  // Use a compatible device
  vkctx.initDevice(compatibleDevices[0], contextInfo);


  // Create example
  HelloVulkan helloVk;

  // Window need to be opened to get the surface on which to draw
  const vk::SurfaceKHR surface = helloVk.getVkSurface(vkctx.m_instance, window);
  vkctx.setGCTQueueWithPresent(surface);

  helloVk.setup(vkctx.m_instance, vkctx.m_device, vkctx.m_physicalDevice,
                vkctx.m_queueGCT.familyIndex);
  helloVk.createSwapchain(surface, SAMPLE_WIDTH, SAMPLE_HEIGHT);
  helloVk.createDepthBuffer();
  helloVk.createRenderPass();
  helloVk.createFrameBuffers();

  // Setup Imgui
  helloVk.initGUI(0);  // Using sub-pass 0

  // Creation of the example
  helloVk.loadScene(nvh::findFile("media/scenes/cornellBox.gltf", defaultSearchPaths, true));


  helloVk.createOffscreenRender();
  helloVk.createDescriptorSetLayout();
  helloVk.createGraphicsPipeline();
  helloVk.createUniformBuffer();
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


  nvmath::vec4f clearColor   = nvmath::vec4f(1, 1, 1, 1.00f);
  bool          useRaytracer = true;


  helloVk.setupGlfwCallbacks(window);
  ImGui_ImplGlfw_InitForVulkan(window, true);

  // Main loop
  while(!glfwWindowShouldClose(window))
  {
    glfwPollEvents();
    if(helloVk.isMinimized())
      continue;

    // Start the Dear ImGui frame
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    // Show UI window.
    if(helloVk.showGui())
    {
      ImGuiH::Panel::Begin();
      ImGui::ColorEdit3("Clear color", reinterpret_cast<float*>(&clearColor));
      ImGui::Checkbox("Ray Tracer mode", &useRaytracer);  // Switch between raster and ray tracing

      renderUI(helloVk);
      ImGui::Text("Application average %.3f ms/frame (%.1f FPS)",
                  1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
      ImGuiH::Control::Info("", "", "(F10) Toggle Pane", ImGuiH::Control::Flags::Disabled);
      ImGuiH::Panel::End();
    }

    // Start rendering the scene
    helloVk.prepareFrame();

    // Start command buffer of this frame
    auto                     curFrame = helloVk.getCurFrame();
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
      if(useRaytracer)
      {
        helloVk.raytrace(cmdBuf, clearColor);
      }
      else
      {
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
      postRenderPassBeginInfo.setRenderPass(helloVk.getRenderPass());
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
  }

  // Cleanup
  helloVk.getDevice().waitIdle();
  helloVk.destroyResources();
  helloVk.destroy();

  vkctx.deinit();

  glfwDestroyWindow(window);
  glfwTerminate();

  return 0;
}
>>>>>>> b7ba7d4fdc4128cefb7ab1d6097cc6d21c37d817
