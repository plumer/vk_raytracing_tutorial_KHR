#ifndef NVCOPY_VK_UTILS_H_
#define NVCOPY_VK_UTILS_H_

#include <glm/glm.hpp>
#include <vector>
#include <vulkan/vulkan.hpp>

#include "types.h"
#include "vk_memory.h"

namespace vkpbr {

template <typename BitType>
bool FlagsMatch(vk::Flags<BitType> candidate, vk::Flags<BitType> desired_flags)
{
    return (candidate & desired_flags) == desired_flags;
}
template <typename BitType>
bool FlagsMatch(vk::Flags<BitType> candidate, BitType desired_flags)
{
    return FlagsMatch(candidate, vk::Flags<BitType>(desired_flags));
}

inline u32 MipLevels(vk::Extent2D extent)
{
    u32 side_width = std::max(extent.width, extent.height);
    return cast_u32(std::floor(std::log2(side_width))) + 1;
}

void CmdGenerateMipmaps(vk::CommandBuffer cmd_buffer, const vk::Image& image,
                        vk::Format image_format, const vk::Extent2D& size, uint32_t mipLevels);

void CmdBarrierImageLayout(vk::CommandBuffer cmd_buffer, vk::Image image,
                           vk::ImageLayout old_layout, vk::ImageLayout new_layout,
                           const vk::ImageSubresourceRange& sub_range);

void CmdBarrierImageLayout(vk::CommandBuffer cmd_buffer, vk::Image image,
                           vk::ImageLayout old_layout, vk::ImageLayout new_layout,
                           vk::ImageAspectFlags aspect_mask);

inline vk::ShaderModule MakeShaderModule(const vk::Device& device, const std::string& binary_code)
{
    auto shader_module_ci = vk::ShaderModuleCreateInfo()
                                .setCodeSize(binary_code.size())
                                .setPCode(reinterpret_cast<const u32*>(binary_code.data()));
    return device.createShaderModule(shader_module_ci);
}

vk::ImageCreateInfo MakeImage2DCreateInfo(
    const vk::Extent2D& size, vk::Format format = vk::Format::eR8G8B8A8Unorm,
    vk::ImageUsageFlags usage = vk::ImageUsageFlagBits::eSampled);

vk::ImageViewCreateInfo MakeImage2DViewCreateInfo(
    const vk::Image& image, vk::Format format = vk::Format::eR8G8B8A8Unorm,
    vk::ImageAspectFlags aspect_flags = vk::ImageAspectFlagBits::eColor, u32 levels = 1,
    const void* p_next_image_view = nullptr);

vk::ImageViewCreateInfo MakeImageViewCreateInfo(const vk::Image&           image,
                                                const vk::ImageCreateInfo& image_ci);

vk::RenderPass MakeRenderPass(vk::Device&                    device,
                              const std::vector<vk::Format>& color_attachment_formats,
                              vk::Format depth_attachment_format, u32 subpass_count = 1,
                              bool clear_color = true, bool clear_depth = true,
                              vk::ImageLayout initial_layout = vk::ImageLayout::eUndefined,
                              vk::ImageLayout final_layout   = vk::ImageLayout::ePresentSrcKHR);

class CommandPool
{
  public:
    CommandPool(const vk::Device& device, u32 queue_family_index,
                vk::CommandPoolCreateFlags flags = vk::CommandPoolCreateFlagBits::eTransient,
                vk::Queue                  default_queue = nullptr);
    ~CommandPool();

    CommandPool(CommandPool const&) = delete;
    CommandPool& operator=(CommandPool const&) = delete;

    vk::CommandBuffer MakeCmdBuffer(
        vk::CommandBufferLevel level = vk::CommandBufferLevel::ePrimary, bool begin = true,
        vk::CommandBufferUsageFlags usage = vk::CommandBufferUsageFlagBits::eOneTimeSubmit,
        const vk::CommandBufferInheritanceInfo* p_inheritance_info = nullptr);

    void SubmitAndWait(vk::ArrayProxy<const vk::CommandBuffer> cmds, vk::Queue queue = nullptr);

  private:
    const vk::Device& device_;
    const vk::Queue   default_queue_;
    // u32               queue_index_ = 0;

    vk::CommandPool cmd_pool_;
};

class DescriptorSetBindings
{
  public:
    DescriptorSetBindings() = default;

    // Wrapper methods from std::vector<T>. `data()` returns pointer-to-const for now.
    void                                  clear() { bindings_.clear(); }
    bool                                  empty() const { return bindings_.empty(); }
    size_t                                size() const { return bindings_.size(); }
    const vk::DescriptorSetLayoutBinding* data() const { return bindings_.data(); }

    void AddBinding(u32 binding_index, vk::DescriptorType type, u32 count,
                    vk::ShaderStageFlags stage_flags,
                    const vk::Sampler*   p_immutable_sampler = nullptr);
    void AddBinding(const vk::DescriptorSetLayoutBinding& binding) { bindings_.push_back(binding); }

    vk::DescriptorType GetType(u32 binding_index) const;
    u32                GetCount(u32 binding_index) const;

    vk::DescriptorSetLayout MakeLayout(vk::Device device,
                                       vk::DescriptorSetLayoutCreateFlags = {}) const;

    vk::DescriptorPool MakePool(vk::Device device, u32 max_sets = 1) const;

    // Methods that make DescriptorSet writes.

    vk::WriteDescriptorSet MakeGeneralDSWrite(vk::DescriptorSet set, u32 binding_index,
                                              u32 array_element = 0) const;

    vk::WriteDescriptorSet MakeWrite(vk::DescriptorSet set, u32 binding_index,
                                     const vk::DescriptorImageInfo* p_image_info,
                                     u32                            array_element = 0) const;
    vk::WriteDescriptorSet MakeWrite(vk::DescriptorSet set, u32 binding_index,
                                     const vk::DescriptorBufferInfo* p_buffer_info,
                                     u32                             array_element = 0) const;
    vk::WriteDescriptorSet MakeWrite(vk::DescriptorSet set, u32 binding_index,
                                     const vk::BufferView* p_texel_buffer_view,
                                     u32                   array_element = 0) const;
    vk::WriteDescriptorSet MakeWrite(vk::DescriptorSet set, u32 binding_index,
                                     const vk::WriteDescriptorSetAccelerationStructureKHR* p_accel,
                                     u32 array_element = 0) const;
    vk::WriteDescriptorSet MakeWrite(
        vk::DescriptorSet set, u32 binding_index,
        const vk::WriteDescriptorSetInlineUniformBlockEXT* p_inline_uniform,
        u32                                                array_element = 0) const;

    // Methods that make DescriptorSet Writers that updates an array of objects.

    // Makes a descriptor set writer with the descriptor count equal to what is recorded.
    vk::WriteDescriptorSet MakeGeneralDSWriteArray(vk::DescriptorSet set, u32 binding_index) const;

    vk::WriteDescriptorSet MakeWriteArray(vk::DescriptorSet set, u32 binding_index,
                                          const vk::DescriptorImageInfo* p_image_info) const;
    vk::WriteDescriptorSet MakeWriteArray(vk::DescriptorSet set, u32 binding_index,
                                          const vk::DescriptorBufferInfo* p_buffer_info) const;
    vk::WriteDescriptorSet MakeWriteArray(vk::DescriptorSet set, u32 binding_index,
                                          const vk::BufferView* p_texel_buffer_view) const;
    vk::WriteDescriptorSet MakeWriteArray(
        vk::DescriptorSet set, u32 binding_index,
        const vk::WriteDescriptorSetAccelerationStructureKHR* p_accel) const;
    vk::WriteDescriptorSet MakeWriteArray(
        vk::DescriptorSet set, u32 binding_index,
        const vk::WriteDescriptorSetInlineUniformBlockEXT* p_inline_uniform) const;

    // An invalid descriptor type within the scope of enum class vk::DescriptorType.
    // By value it is equal to VK_DESCRIPTOR_TYPE_MAX_ENUM.
    static constexpr vk::DescriptorType kInvalidType =
        static_cast<vk::DescriptorType>(VK_DESCRIPTOR_TYPE_MAX_ENUM);

  private:
    std::vector<vk::DescriptorSetLayoutBinding> bindings_;
};

struct ContextCreateInfo {

    ContextCreateInfo(bool use_validation = true);

    void SetVersion(int major, int minor);
    void AddInstanceExtension(const char* name, bool optional = false);
    void AddInstanceExtension(std::vector<const char*> names, bool optional = false);
    void AddInstanceLayer(const char* name, bool optional = false);
    void AddInstanceLayer(std::vector<const char*> names, bool optional = false);
    void AddDeviceExtension(const char* name, bool optional = false,
                            void* p_feature_struct = nullptr);
    void AddDeviceExtension(std::vector<const char*> names, bool optional = false,
                            std::vector<void*> p_features = {});

    void RemoveInstanceExtension(const char* name);
    void RemoveInstanceLayer(const char* name);
    void RemoveDeviceExtension(const char* name);

    // Members
    // ----------------------------------------------------------------------------------------
    bool        use_device_groups            = false;
    u32         compatible_device_index      = 0;
    const char* app_engine_name              = "nvpro-sample";
    const char* app_title                    = "nvpro-sample";
    bool        disable_robust_buffer_access = true;
    bool        verbose_compatible_devices   = true;
    bool        verbose_used                 = true;
    bool        verbose_available            = true;

    // A unified interface for specifying instance layers, instance / device extensions.
    struct Entry {
        Entry(const char* n, bool o, void* feature = nullptr, u32 ver = 0)
            : name(n)
            , optional(o)
            , p_feature_struct(feature)
            , version(ver)
        {}
        const char* name             = nullptr;
        bool        optional         = false;
        void*       p_feature_struct = nullptr;
        u32         version          = 0;
    };

    int api_major = 1;
    int api_minor = 1;

    std::vector<Entry> instance_layers;
    std::vector<Entry> instance_extensions;
    std::vector<Entry> device_extensions;
    void*              device_create_info_ext = nullptr;
};

class Context
{
  public:
    Context(Context const&) = delete;
    Context& operator=(const Context&) = delete;

    Context() = default;

    bool Init(const ContextCreateInfo& context_ci);
    void DeInit();

    bool InitInstance(const ContextCreateInfo& context_ci);
    bool InitDevice(u32 device_index, const ContextCreateInfo& context_ci);

    const vk::Device& Device() const { return device_; }

    std::vector<u32>                GetCompatibleDevices(const ContextCreateInfo& context_ci) const;
    std::vector<vk::PhysicalDevice> GetGpus() const
    {
        return instance_.enumeratePhysicalDevices(static_loader_);
    }
    std::vector<vk::PhysicalDeviceGroupProperties> GetGpuGroups() const
    {
        return instance_.enumeratePhysicalDeviceGroups();
    }
    std::vector<vk::ExtensionProperties> GetInstanceExtensions() const
    {
        return vk::enumerateInstanceExtensionProperties(nullptr, static_loader_);
    }
    std::vector<vk::LayerProperties> GetInstanceLayers() const
    {
        return vk::enumerateInstanceLayerProperties(static_loader_);
    }
    std::vector<vk::ExtensionProperties> GetGpuExtensions(vk::PhysicalDevice gpu) const
    {
        return gpu.enumerateDeviceExtensionProperties(nullptr, static_loader_);
    }

    bool HasMandatoryExtensions(vk::PhysicalDevice gpu, const ContextCreateInfo& context_ci,
                                bool verbose) const;

    // Ensures thet G/C/T queue can present to the provided surface. Sets the GCT queue to be such
    // a queue the GPU supports it.
    // Returns false if fails to set.
    bool SetGCTQueueWithPresent(vk::SurfaceKHR surface);

    u32 GetQueueFamily(vk::QueueFlags flags_supported, vk::QueueFlags flags_disabled = {},
                       vk::SurfaceKHR surface = nullptr);

    // Checks if the context has the optional extension activated.
    bool HasDeviceExtension(const char* name) const;
    bool HasInstanceExtension(const char* name) const;

    vk::Instance       instance() const { return instance_; }
    vk::Device         device() const { return device_; }
    vk::PhysicalDevice gpu() const { return gpu_; }
    vk::Queue          GetGctQueue() const { return queue_graphics_compute_transfer_.queue; }
    u32 GetGctQueueFamilyIndex() const { return queue_graphics_compute_transfer_.family_index; }

  private:
    vk::Instance       instance_;
    vk::Device         device_;
    vk::PhysicalDevice gpu_;
    struct GpuInfo {
        GpuInfo()                                                = default;
        VkPhysicalDeviceMemoryProperties       memory_properties = {};
        std::vector<vk::QueueFamilyProperties> queue_properties;

        vk::PhysicalDeviceFeatures         features_100;
        vk::PhysicalDeviceVulkan11Features features_110;
        vk::PhysicalDeviceVulkan12Features features_120;

        vk::PhysicalDeviceProperties         properties_100;
        vk::PhysicalDeviceVulkan11Properties properties_110;
        vk::PhysicalDeviceVulkan12Properties properties_120;
    } gpu_info_;

    struct Queue {
        vk::Queue queue;
        u32       family_index = ~0;
    };
    Queue queue_graphics_compute_transfer_;
    Queue queue_transfer_;
    Queue queue_compute_;

    std::vector<const char*> used_instance_layers_;
    std::vector<const char*> used_instance_extensions_;
    std::vector<const char*> used_device_extensions_;

    PFN_vkCreateDebugUtilsMessengerEXT  fp_createDebugUtilsMessengerEXT_ = nullptr;
    PFN_vkDestroyDebugUtilsMessengerEXT fp_destroyDebugUtilsMessengerEXT = nullptr;
    VkDebugUtilsMessengerEXT            debug_messenger_                 = VK_NULL_HANDLE;

    void InitDebugUtils();

    vk::Result FillFilterNameArray(std::vector<const char*>*                    used,
                                   const std::vector<vk::LayerProperties>&      properties,
                                   const std::vector<ContextCreateInfo::Entry>& requested);

    vk::Result FillFilterNameArray(std::vector<const char*>*                    used,
                                   const std::vector<vk::ExtensionProperties>&  properties,
                                   const std::vector<ContextCreateInfo::Entry>& requested,
                                   std::vector<void*>&                          feature_structs);

    bool CheckEntryArray(const std::vector<vk::ExtensionProperties>&  properties,
                         const std::vector<ContextCreateInfo::Entry>& requested,
                         bool                                         verbose) const;

    static void InitGpuInfo(GpuInfo* info, vk::PhysicalDevice gpu, u32 version_major,
                            u32 version_minor);

    static vk::DispatchLoaderStatic static_loader_;
};

}  // namespace vkpbr

#endif  // !NVCOPY_VK_UTILS_H_
