#ifndef NVCOPY_VK_UTILS_H_
#define NVCOPY_VK_UTILS_H_

#include <glm/glm.hpp>
#include <vector>
#include <set>
#include <vulkan/vulkan.hpp>
#include <cstdarg>

#include "types.h"
#include "vk_memory.h"

namespace vkpbr {

template<typename ... Args>
std::string Format(const char * format, Args ... args) {
    int length = std::snprintf(nullptr, 0, format, args...) + 1;
    if (length <= 0) {
        throw std::runtime_error("Error during formatting.");
    }
    std::unique_ptr<char[]> buf(new char[length]);
    std::snprintf(buf.get(), length, format, args...);
    return std::string(buf.get(), buf.get() + length - 1);
}

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

inline void CmdBarrierImageLayout(vk::CommandBuffer cmd_buffer, vk::Image image,
                           vk::ImageLayout old_layout, vk::ImageLayout new_layout) {
    CmdBarrierImageLayout(cmd_buffer, image, old_layout, new_layout, vk::ImageAspectFlagBits::eColor);
}

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

vk::ImageCreateInfo MakeImage2DCreateInfo(const vk::Extent2D& size, vk::Format format,
                                          vk::ImageUsageFlags usage, bool use_mipmaps);

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

#define USE_NV_CONTEXT

struct ContextCreateInfo {
    ContextCreateInfo(bool bUseValidation = true);

    void SetVersion(int major, int minor);

    void AddInstanceExtension(const char* name, bool optional = false);
    void AddInstanceExtension(const std::vector<const char*>& names)
    {
        for (const char* name : names)
            AddInstanceExtension(name, /*optional = */ false);
    }
    void AddInstanceLayer(const char* name, bool optional = false);
    // version = 0: don't care, otherwise check against equality (useful for provisional exts)
    void AddDeviceExtension(const char* name, bool optional = false, void* pFeatureStruct = nullptr,
                            uint32_t version = 0);
    void AddDeviceExtension(const std::vector<const char*>& names)
    {
        for (const char *name : names)
            AddDeviceExtension(name, /*optional =*/false);
    }
    

    void removeInstanceExtension(const char* name);
    void removeInstanceLayer(const char* name);
    void removeDeviceExtension(const char* name);


    // Configure additional device creation with these variables and functions

    // use device groups
    bool useDeviceGroups = false;

    // which compatible device or device group to pick
    // only used by All-in-one Context::init(...)
    uint32_t compatibleDeviceIndex = 0;

    // instance properties
    const char* appEngine = "nvpro-sample";
    const char* appTitle  = "nvpro-sample";

    // may impact performance hence disable by default
    bool disableRobustBufferAccess = true;

    // Information printed at Context::init time
    bool verboseCompatibleDevices = true;
    bool verboseUsed              = true;  // Print what is used
    bool verboseAvailable         =        // Print what is available
#ifdef _DEBUG
        true;
#else
        false;
#endif

    struct Entry {
        Entry(const char* entryName, bool isOptional = false, void* pointerFeatureStruct = nullptr,
              uint32_t checkVersion = 0)
            : name(entryName)
            , optional(isOptional)
            , pFeatureStruct(pointerFeatureStruct)
            , version(checkVersion)
        {}
        const char* name{nullptr};
        bool        optional{false};
        void*       pFeatureStruct{nullptr};
        uint32_t    version{0};
    };

    int apiMajor = 1;
    int apiMinor = 1;

    using EntryArray = std::vector<Entry>;
    EntryArray instanceLayers;
    EntryArray instanceExtensions;
    EntryArray deviceExtensions;
    void*      deviceCreateInfoExt   = nullptr;
    void*      instanceCreateInfoExt = nullptr;
};

//////////////////////////////////////////////////////////////////////////
/**
# class nvvk::Context

Context class helps creating the Vulkan instance and to choose the logical device for the mandatory
extensions. First is to fill the `ContextCreateInfo` structure, then call:

~~~ C++
  // Creating the Vulkan instance and device
  nvvk::ContextCreateInfo ctxInfo;
  ... see above ...

  nvvk::Context vkctx;
  vkctx.init(ctxInfo);

  // after init the ctxInfo is no longer needed
~~~

At this point, the class will have created the `VkInstance` and `VkDevice` according to the
information passed. It will also keeps track or have query the information of:

* Physical Device information that you can later query : `PhysicalDeviceInfo` in which lots of
`VkPhysicalDevice...` are stored
* `VkInstance` : the one instance being used for the programm
* `VkPhysicalDevice` : physical device(s) used for the logical device creation. In case of more than
one physical device, we have a std::vector for this purpose...
* `VkDevice` : the logical device instanciated
* `VkQueue` : we will enumerate all the available queues and make them available in `nvvk::Context`.
Some queues are specialized, while other are for general purpose (most of the time, only one can
handle everything, while other queues are more specialized). We decided to make them all available
in some explicit way :
 * `Queue m_queueGCT` : Graphics/Compute/Transfer Queue + family index
 * `Queue m_queueT` : async Transfer Queue + family index
 * `Queue m_queueC` : Compute Queue + family index
* maintains what extensions are finally available
* implicitly hooks up the debug callback

## Choosing the device
When there are multiple devices, the `init` method is choosing the first compatible device
available, but it is also possible the choose another one.
~~~ C++
  vkctx.initInstance(deviceInfo);
  // Find all compatible devices
  auto compatibleDevices = vkctx.getCompatibleDevices(deviceInfo);
  assert(!compatibleDevices.empty());

  // Use first compatible device
  vkctx.initDevice(compatibleDevices[0], deviceInfo);
~~~

## Multi-GPU

When multiple graphic cards should be used as a single device, the
`ContextCreateInfo::useDeviceGroups` need to be set to `true`. The above methods will transparently
create the `VkDevice` using `VkDeviceGroupDeviceCreateInfo`. Especially in the context of NVLink
connected cards this is useful.


*/
class Context
{
  public:
    Context(Context const&) = delete;
    Context& operator=(Context const&) = delete;

    Context() = default;

    using NameArray = std::vector<const char*>;

    // Vulkan == 1.1 used individual structs
    // Vulkan >= 1.2  have per-version structs
    struct Features11Old {
        VkPhysicalDeviceMultiviewFeatures multiview{
            VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_MULTIVIEW_FEATURES};
        VkPhysicalDevice16BitStorageFeatures t16BitStorage{
            VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_16BIT_STORAGE_FEATURES};
        VkPhysicalDeviceSamplerYcbcrConversionFeatures samplerYcbcrConversion{
            VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_SAMPLER_YCBCR_CONVERSION_FEATURES};
        VkPhysicalDeviceProtectedMemoryFeatures protectedMemory{
            VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROTECTED_MEMORY_FEATURES};
        VkPhysicalDeviceShaderDrawParameterFeatures drawParameters{
            VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_SHADER_DRAW_PARAMETER_FEATURES};
        VkPhysicalDeviceVariablePointerFeatures variablePointers{
            VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VARIABLE_POINTER_FEATURES};

        Features11Old()
        {
            multiview.pNext              = &t16BitStorage;
            t16BitStorage.pNext          = &samplerYcbcrConversion;
            samplerYcbcrConversion.pNext = &protectedMemory;
            protectedMemory.pNext        = &drawParameters;
            drawParameters.pNext         = &variablePointers;
            variablePointers.pNext       = nullptr;
        }

        void read(const VkPhysicalDeviceVulkan11Features& features11)
        {
            multiview.multiview                    = features11.multiview;
            multiview.multiviewGeometryShader      = features11.multiviewGeometryShader;
            multiview.multiviewTessellationShader  = features11.multiviewTessellationShader;
            t16BitStorage.storageBuffer16BitAccess = features11.storageBuffer16BitAccess;
            t16BitStorage.storageInputOutput16     = features11.storageInputOutput16;
            t16BitStorage.storagePushConstant16    = features11.storagePushConstant16;
            t16BitStorage.uniformAndStorageBuffer16BitAccess =
                features11.uniformAndStorageBuffer16BitAccess;
            samplerYcbcrConversion.samplerYcbcrConversion = features11.samplerYcbcrConversion;
            protectedMemory.protectedMemory               = features11.protectedMemory;
            drawParameters.shaderDrawParameters           = features11.shaderDrawParameters;
            variablePointers.variablePointers             = features11.variablePointers;
            variablePointers.variablePointersStorageBuffer =
                features11.variablePointersStorageBuffer;
        }

        void write(VkPhysicalDeviceVulkan11Features& features11)
        {
            features11.multiview                   = multiview.multiview;
            features11.multiviewGeometryShader     = multiview.multiviewGeometryShader;
            features11.multiviewTessellationShader = multiview.multiviewTessellationShader;
            features11.storageBuffer16BitAccess    = t16BitStorage.storageBuffer16BitAccess;
            features11.storageInputOutput16        = t16BitStorage.storageInputOutput16;
            features11.storagePushConstant16       = t16BitStorage.storagePushConstant16;
            features11.uniformAndStorageBuffer16BitAccess =
                t16BitStorage.uniformAndStorageBuffer16BitAccess;
            features11.samplerYcbcrConversion = samplerYcbcrConversion.samplerYcbcrConversion;
            features11.protectedMemory        = protectedMemory.protectedMemory;
            features11.shaderDrawParameters   = drawParameters.shaderDrawParameters;
            features11.variablePointers       = variablePointers.variablePointers;
            features11.variablePointersStorageBuffer =
                variablePointers.variablePointersStorageBuffer;
        }
    };
    struct Properties11Old {
        VkPhysicalDeviceMaintenance3Properties maintenance3{
            VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_MAINTENANCE_3_PROPERTIES};
        VkPhysicalDeviceIDProperties deviceID{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ID_PROPERTIES};
        VkPhysicalDeviceMultiviewProperties multiview{
            VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_MULTIVIEW_PROPERTIES};
        VkPhysicalDeviceProtectedMemoryProperties protectedMemory{
            VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROTECTED_MEMORY_PROPERTIES};
        VkPhysicalDevicePointClippingProperties pointClipping{
            VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_POINT_CLIPPING_PROPERTIES};
        VkPhysicalDeviceSubgroupProperties subgroup{
            VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_SUBGROUP_PROPERTIES};

        Properties11Old()
        {
            maintenance3.pNext    = &deviceID;
            deviceID.pNext        = &multiview;
            multiview.pNext       = &protectedMemory;
            protectedMemory.pNext = &pointClipping;
            pointClipping.pNext   = &subgroup;
            subgroup.pNext        = nullptr;
        }

        void write(VkPhysicalDeviceVulkan11Properties& properties11)
        {
            memcpy(properties11.deviceLUID, deviceID.deviceLUID, sizeof(properties11.deviceLUID));
            memcpy(properties11.deviceUUID, deviceID.deviceUUID, sizeof(properties11.deviceUUID));
            memcpy(properties11.driverUUID, deviceID.driverUUID, sizeof(properties11.driverUUID));
            properties11.deviceLUIDValid                   = deviceID.deviceLUIDValid;
            properties11.deviceNodeMask                    = deviceID.deviceNodeMask;
            properties11.subgroupSize                      = subgroup.subgroupSize;
            properties11.subgroupSupportedStages           = subgroup.supportedStages;
            properties11.subgroupSupportedOperations       = subgroup.supportedOperations;
            properties11.subgroupQuadOperationsInAllStages = subgroup.quadOperationsInAllStages;
            properties11.pointClippingBehavior             = pointClipping.pointClippingBehavior;
            properties11.maxMultiviewViewCount             = multiview.maxMultiviewViewCount;
            properties11.maxMultiviewInstanceIndex         = multiview.maxMultiviewInstanceIndex;
            properties11.protectedNoFault                  = protectedMemory.protectedNoFault;
            properties11.maxPerSetDescriptors              = maintenance3.maxPerSetDescriptors;
            properties11.maxMemoryAllocationSize           = maintenance3.maxMemoryAllocationSize;
        }
    };

    // This struct holds all core feature information for a physical device
    struct PhysicalDeviceInfo {
        VkPhysicalDeviceMemoryProperties     memoryProperties{};
        std::vector<VkQueueFamilyProperties> queueProperties;

        VkPhysicalDeviceFeatures         features10{};
        VkPhysicalDeviceVulkan11Features features11{
            VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_1_FEATURES};
        VkPhysicalDeviceVulkan12Features features12{
            VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_2_FEATURES};

        VkPhysicalDeviceProperties         properties10{};
        VkPhysicalDeviceVulkan11Properties properties11{
            VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_1_PROPERTIES};
        VkPhysicalDeviceVulkan12Properties properties12{
            VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_2_PROPERTIES};
    };

    struct Queue {
        VkQueue  queue       = VK_NULL_HANDLE;
        uint32_t familyIndex = ~0;
        uint32_t queueIndex  = ~0;

        operator VkQueue() const { return queue; }
        operator uint32_t() const { return familyIndex; }
    };


    vk::Instance         m_instance = nullptr;
    vk::Device           m_device = nullptr;
    VkPhysicalDevice   m_physicalDevice{VK_NULL_HANDLE};
    PhysicalDeviceInfo m_physicalInfo;

    vk::Instance       Instance() const { return vk::Instance(m_instance); }
    vk::PhysicalDevice Gpu() const { return vk::PhysicalDevice(m_physicalDevice); }
    vk::Device         Device() const { return vk::Device(m_device); }

    // All the queues (if present) is distinct from each other
    Queue m_queueGCT;  // for Graphics/Compute/Transfer (must exist)
    Queue m_queueT;    // for pure async Transfer Queue (can exist, supports at least transfer)
    Queue m_queueC;    // for async Compute (can exist, supports at least compute)
    // Ensures the GCT queue can present to the provided surface (return false if fails to set)
    bool SetGCTQueueWithPresent(VkSurfaceKHR surface);
    u32  GetGctQueueFamilyIndex() const { return m_queueGCT.familyIndex; }

    operator VkDevice() const { return m_device; }

    // All-in-one instance and device creation
    bool Init(const ContextCreateInfo& info);
    void DeInit();

    // Individual object creation
    bool InitInstance(const ContextCreateInfo& info);
    // deviceIndex is an index either into getPhysicalDevices or getPhysicalDeviceGroups
    // depending on info.useDeviceGroups
    bool InitDevice(uint32_t deviceIndex, const ContextCreateInfo& info);

    // Helpers
    std::vector<int>              GetCompatibleDevices(const ContextCreateInfo& info);
    std::vector<VkPhysicalDevice> GetGpus();
    std::vector<VkPhysicalDeviceGroupProperties> GetGpuGroups();
    std::vector<VkExtensionProperties>           GetInstanceExtensions();
    std::vector<VkLayerProperties>               GetInstanceLayers();
    std::vector<VkExtensionProperties> GetGpuExtensions(VkPhysicalDevice physicalDevice);
    bool HasMandatoryExtensions(VkPhysicalDevice physicalDevice, const ContextCreateInfo& info,
                                bool bVerbose);


    uint32_t getQueueFamily(VkQueueFlags flagsSupported, VkQueueFlags flagsDisabled = 0,
                            VkSurfaceKHR surface = VK_NULL_HANDLE);

    // true if the context has the optional extension activated
    bool hasDeviceExtension(const char* name) const;
    bool hasInstanceExtension(const char* name) const;

    void ignoreDebugMessage(int32_t msgID) { m_dbgIgnoreMessages.insert(msgID); }

  private:
    static VKAPI_ATTR VkBool32 VKAPI_CALL debugMessengerCallback(
        VkDebugUtilsMessageSeverityFlagBitsEXT      messageSeverity,
        VkDebugUtilsMessageTypeFlagsEXT             messageType,
        const VkDebugUtilsMessengerCallbackDataEXT* callbackData, void* userData);

    NameArray m_usedInstanceLayers;
    NameArray m_usedInstanceExtensions;
    NameArray m_usedDeviceExtensions;

    // New Debug system
    PFN_vkCreateDebugUtilsMessengerEXT  m_createDebugUtilsMessengerEXT  = nullptr;
    PFN_vkDestroyDebugUtilsMessengerEXT m_destroyDebugUtilsMessengerEXT = nullptr;
    VkDebugUtilsMessengerEXT            m_dbgMessenger                  = nullptr;

    std::set<int32_t> m_dbgIgnoreMessages;

    void initDebugUtils();

    VkResult    fillFilteredNameArray(Context::NameArray&                   used,
                                      const std::vector<VkLayerProperties>& properties,
                                      const ContextCreateInfo::EntryArray&  requested);
    VkResult    fillFilteredNameArray(Context::NameArray&                       used,
                                      const std::vector<VkExtensionProperties>& properties,
                                      const ContextCreateInfo::EntryArray&      requested,
                                      std::vector<void*>&                       featureStructs);
    bool        checkEntryArray(const std::vector<VkExtensionProperties>& properties,
                                const ContextCreateInfo::EntryArray& requested, bool bVerbose);
    static void initPhysicalInfo(PhysicalDeviceInfo& info, VkPhysicalDevice physicalDevice,
                                 uint32_t versionMajor, uint32_t versionMinor);
};

#ifdef USE_NV_CONTEXT

#else
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
    void*              instance_create_info_ext = nullptr;
};

class Context
{
  public:
    Context(Context const&) = delete;
    Context& operator=(const Context&) = delete;

    Context() = default;

    // Creates an instance, looks for compatible GPUs supporting required extensions, and creates a
    // logical device.
    bool Init(const ContextCreateInfo& context_ci);
    void DeInit();

    /**
     * \brief Makes the instance, given application name, engine name, and VK version
     * number. All required layers and extensions in the context create info are checked and cached
     * before the instance is created.
     * Creates a debug util messenger if VK_EXT_DEBUG_UTILS is specified in the argument.
     */
    bool InitInstance(const ContextCreateInfo& context_ci);

    /**
     * Creates the device from the given GPU index (i.e., GetGpus()[device_index]) and
     * retrieves queues. Features and properties under the Vulkan API version specified in
     * `context_ci` are retrieved and used to create the device.
     *
     * GPU extensions required in context_ci are ensured to be supported by the specified GPU.

     * \param context_ci specifies extensions required to be supported. \return
     * vk::Result::eSuccess if all extensions are supported.
     */
    bool InitDevice(u32 device_index, const ContextCreateInfo& context_ci);

    const vk::Device&  Device() const { return device_; }
    vk::Instance       Instance() const { return instance_; }
    vk::PhysicalDevice Gpu() const { return gpu_; }
    vk::Queue          GetGctQueue() const { return queue_graphics_compute_transfer_.queue; }
    u32 GetGctQueueFamilyIndex() const { return queue_graphics_compute_transfer_.family_index; }

    // Retrieves the set of GPUs that supports all extensions specified in the context_ci.
    std::vector<u32> GetCompatibleDevices(const ContextCreateInfo& context_ci) const;

    // Retrieves the set of GPUs connected to the host machine. An instance is implicitly required.
    std::vector<vk::PhysicalDevice> GetGpus() const
    {
        return instance_.enumeratePhysicalDevices(static_loader_);
    }
    // Retrieves the set of GPU groups connected to the host machine. An instance in implicitly
    // needed.
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

    // Checks if the extensions required by context_ci are supported by the given GPU.
    bool HasMandatoryExtensions(vk::PhysicalDevice gpu, const ContextCreateInfo& context_ci,
                                bool verbose) const;

    // Ensures thet G/C/T queue can present to the provided surface. Sets the GCT queue to be such
    // a queue the GPU supports it.
    // Returns false if fails to set.
    bool SetGCTQueueWithPresent(vk::SurfaceKHR surface);

    // Checks if the context has the optional extension activated.
    bool HasDeviceExtension(const char* name) const;
    bool HasInstanceExtension(const char* name) const;

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

    // Checks if all requested layers are available in the supported properties. If so, writes the
    // names of all such layers into `used`.
    vk::Result FillFilterNameArray(std::vector<const char*>*                    used,
                                   const std::vector<vk::LayerProperties>&      properties,
                                   const std::vector<ContextCreateInfo::Entry>& requested);

    // Selects supported properties from requested ones and returns them in `used`.
    vk::Result FillFilterNameArray(std::vector<const char*>*                    used,
                                   const std::vector<vk::ExtensionProperties>&  properties,
                                   const std::vector<ContextCreateInfo::Entry>& requested,
                                   std::vector<void*>*                          feature_structs);

    // Checks if all requested extensions are available in given supported properties.
    bool CheckEntryArray(const std::vector<vk::ExtensionProperties>&  supported_properties,
                         const std::vector<ContextCreateInfo::Entry>& requested,
                         bool                                         verbose) const;

    /**
     * \brief Writes all supported features and properties from a GPU into `info`.
     * \param version_major, version_minor: Vulkan version being used.
     * \return info: GPU info containing the supported features.
     */
    static void InitGpuInfo(GpuInfo* info, vk::PhysicalDevice gpu, u32 version_major,
                            u32 version_minor);

    static vk::DispatchLoaderStatic static_loader_;
};
#endif

template <typename T, typename U>
class ZippedVectorIterator
{
  private:
    typename std::vector<T>::iterator first_iter_;
    typename std::vector<U>::iterator second_iter_;

  public:
    std::pair<const T&, const U&> operator*() const
    {
        const T& t = *first_iter_;
        const U& u = *second_iter_;
        return std::pair<const T&, const U&>(t, u);
    }

    // Post-increment
    ZippedVectorIterator operator++()
    {
        ZippedVectorIterator old_i = *this;
        first_iter_++;
        second_iter_++;
        return old_i;
    }
    ZippedVectorIterator& operator++(int)
    {
        first_iter_++;
        second_iter_++;
        return *this;
    }

    bool operator==(const ZippedVectorIterator& rhs) const
    {
        return first_iter_ == rhs.first_iter_ && second_iter_ == rhs.second_iter_;
    }
    bool operator!=(const ZippedVectorIterator& rhs) const { return !(*this == rhs); }
};

template <typename T, typename U>
class ZippedVectors
{
  public:
    std::vector<T>& first_;
    std::vector<U>& second_;
    using iterator = ZippedVectorIterator<T, U>;

    ZippedVectors(std::vector<T>& first, std::vector<U>& second)
        : first_(first)
        , second_(second)
    {
        assert(first.size() == second.size());
    }
    iterator begin()
    {
        iterator i;
        i.first_iter_  = first_.begin();
        i.second_iter_ = second_.begin();
        return i;
    }
    iterator end()
    {
        iterator i;
        i.first_iter_  = first_.end();
        i.second_iter_ = second_.end();
        return i;
    }
};

template <typename T, typename U>
ZippedVectors<T, U> zip(std::vector<T>& a, std::vector<U>& b)
{
    return ZippedVectors<T, U>(a, b);
}


}  // namespace vkpbr

#endif  // !NVCOPY_VK_UTILS_H_
