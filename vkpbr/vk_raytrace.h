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
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``Bvh IS'' AND ANY
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

/**

# class nvvk::RaytracingBuilderKHR

Base functionality of raytracing

This class acts as an owning container for a single top-level acceleration
structure referencing any number of bottom-level acceleration structures.
We provide functions for building (on the device) an array of BLASs and a
single TLAS from vectors of BlasInput and Instance, respectively, and
a destroy function for cleaning up the created acceleration structures.

Generally, we reference BLASs by their index in the stored BLAS array,
rather than using raw device pointers as the pure Vulkan acceleration
structure API uses.

This class does not support replacing acceleration structures once
built, but you can update the acceleration structures. For educational
purposes, this class prioritizes (relative) understandability over
performance, so vkQueueWaitIdle is implicitly used everywhere.

# Setup and Usage
~~~~ C++
// Borrow a VkDevice and memory allocator pointer (must remain
// valid throughout our use of the ray trace builder), and
// instantiate an unspecified queue of the given family for use.
m_rtBuilder.setup(device, memoryAllocator, queueIndex);

// You create a vector of RayTracingBuilderKHR::BlasInput then
// pass it to buildBlas.
std::vector<RayTracingBuilderKHR::BlasInput> inputs = // ...
m_rtBuilder.buildBlas(inputs);

// You create a vector of RaytracingBuilder::Instance and pass to
// buildTlas. The blasId member of each instance must be below
// inputs.size() (above).
std::vector<RayTracingBuilderKHR::Instance> instances = // ...
m_rtBuilder.buildTlas(instances);

// Retrieve the handle to the acceleration structure.
constexpr VkAccelerationStructureKHR tlas = m.rtBuilder.getAccelerationStructure()
~~~~
*/

#include <mutex>
#include <vulkan/vulkan.h>

//#define NVVK_ALLOC_DEDICATED
//#include <nvvk/allocator_vk.hpp>
//#include <nvvk/commands_vk.hpp>
#include <nvvk/debug_util_vk.hpp>
//#include "nvmath/nvmath.h"
#include "vk_memory.h"
#include <glm/glm.hpp>

#if VK_KHR_acceleration_structure


namespace vkpbr {
struct RaytracingBuilderKHR {
    RaytracingBuilderKHR(RaytracingBuilderKHR const&) = delete;
    RaytracingBuilderKHR& operator=(RaytracingBuilderKHR const&) = delete;

    RaytracingBuilderKHR() = default;

    // Inputs used to build Bottom-level acceleration structure.
    // You manage the lifetime of the buffer(s) referenced by the
    // VkAccelerationStructureGeometryKHRs within. In particular, you must
    // make sure they are still valid and not being modified when the BLAS
    // is built or updated.
    struct BlasInput {
        // Data used to build acceleration structure geometry
        std::vector<vk::AccelerationStructureGeometryKHR>       asGeometry;
        std::vector<vk::AccelerationStructureBuildRangeInfoKHR> asBuildOffsetInfo;
    };

  private:
    // Bottom-level acceleration structure, along with the information needed to re-build it.
    struct BlasEntry {
        // User-provided input.
        BlasInput input;

        // VkAccelerationStructureKHR plus extra info needed for our memory allocator.
        // The RaytracingBuilderKHR that created this DOES destroy it when destroyed.
        UniqueMemoryAccelStruct as;

        // Additional parameters for acceleration structure builds
        vk::BuildAccelerationStructureFlagsKHR flags;

        BlasEntry() = default;
        BlasEntry(BlasInput input_)
            : input(std::move(input_))
            , as()
        {}
    };

  public:
    //--------------------------------------------------------------------------------------------------
    // Initializing the allocator and querying the raytracing properties
    //

    void setup(const vk::Device& device, UniqueMemoryAllocator* allocator, uint32_t queueIndex);

    // This is an instance of a BLAS
    struct Instance {
        uint32_t                     blasId{0};      // Index of the BLAS in m_blas
        uint32_t                     instanceId{0};  // Instance Index (gl_InstanceID)
        uint32_t                     hitGroupId{0};  // Hit group index in the SBT
        uint32_t                     mask{0xFF};  // Visibility mask, will be AND-ed with ray mask
        vk::GeometryInstanceFlagsKHR flags =
            vk::GeometryInstanceFlagBitsKHR::eTriangleFacingCullDisable;
        glm::mat4 transform{1.0f};  // Identity
    };

    //--------------------------------------------------------------------------------------------------
    // Destroying all allocations
    //

    void destroy();

    // Returning the constexprructed top-level acceleration structure
    vk::AccelerationStructureKHR getAccelerationStructure() const { return m_tlas.as.handle; }

    //--------------------------------------------------------------------------------------------------
    // Create all the BLAS from the vector of BlasInput
    // - There will be one BLAS per input-vector entry
    // - There will be as many BLAS as input.size()
    // - The resulting BLAS (along with the inputs used to build) are stored in m_blas,
    //   and can be referenced by index
    void buildBlas(const std::vector<RaytracingBuilderKHR::BlasInput>& input,
                   vk::BuildAccelerationStructureFlagsKHR              flags =
                       vk::BuildAccelerationStructureFlagBitsKHR::ePreferFastTrace);


    //--------------------------------------------------------------------------------------------------
    // Convert an Instance object into a VkAccelerationStructureInstanceKHR

    vk::AccelerationStructureInstanceKHR instanceToVkGeometryInstanceKHR(
        const Instance& instance);

    //--------------------------------------------------------------------------------------------------
    // Creating the top-level acceleration structure from the vector of Instance
    // - See struct of Instance
    // - The resulting TLAS will be stored in m_tlas
    // - update is to rebuild the Tlas with updated matrices
    void buildTlas(const std::vector<Instance>&       instances,
                   vk::BuildAccelerationStructureFlagsKHR flags =
                       vk::BuildAccelerationStructureFlagBitsKHR::ePreferFastTrace,
                   bool update = false);

    //--------------------------------------------------------------------------------------------------
    // Refit BLAS number blasIdx from updated buffer contents.
    //
    void updateBlas(uint32_t blasIdx);

  private:
    // Top-level acceleration structure
    struct Tlas {
        UniqueMemoryAccelStruct                as;
        vk::BuildAccelerationStructureFlagsKHR flags;
    };

    //--------------------------------------------------------------------------------------------------
    // Vector containing all the BLASes built in buildBlas (and referenced by the TLAS)
    std::vector<BlasEntry> m_blas;
    // Top-level acceleration structure
    Tlas m_tlas;
    // Instance buffer containing the matrices and BLAS ids
    vkpbr::UniqueMemoryBuffer m_instBuffer;

    vk::Device m_device;
    uint32_t   m_queueIndex{0};

    UniqueMemoryAllocator* m_alloc = nullptr;
    nvvk::DebugUtil        m_debug;
};

}  // namespace vkpbr

namespace vkrt {
using BuildBvhFlagBits = vk::BuildAccelerationStructureFlagBitsKHR;
using BuildBvhFlags    = vk::BuildAccelerationStructureFlagsKHR;
using BuildBvhMode     = vk::BuildAccelerationStructureModeKHR;

using BvhBuildGeometryInfo = vk::AccelerationStructureBuildGeometryInfoKHR;
using BvhBuildRangeInfo    = vk::AccelerationStructureBuildRangeInfoKHR;
using BvhBuildSizesInfo    = vk::AccelerationStructureBuildSizesInfoKHR;
using BvhBuildType         = vk::AccelerationStructureBuildTypeKHR;
using BvhCompatibility     = vk::AccelerationStructureCompatibilityKHR;
using BvhCreateInfo        = vk::AccelerationStructureCreateInfoKHR;
using BvhCreateFlagBits    = vk::AccelerationStructureCreateFlagBitsKHR;
using BvhCreateFlags       = vk::AccelerationStructureCreateFlagsKHR;
using BvhDeviceAddrInfo    = vk::AccelerationStructureDeviceAddressInfoKHR;
using BvhGeometryAabbsData = vk::AccelerationStructureGeometryAabbsDataKHR;

using BvhInstanceKHR = vk::AccelerationStructureInstanceKHR;
using BvhType        = vk::AccelerationStructureTypeKHR;

using CopyBvhInfo = vk::CopyAccelerationStructureInfoKHR;
using CopyBvhMode = vk::CopyAccelerationStructureModeKHR;

using RtShaderGroupCreateInfo = vk::RayTracingShaderGroupCreateInfoKHR;
using RtShaderGroupType       = vk::RayTracingShaderGroupTypeKHR;
using RtPipelineCreateInfo    = vk::RayTracingPipelineCreateInfoKHR;

using GpuRtPipelineProperties = vk::PhysicalDeviceRayTracingPipelinePropertiesKHR;

using WriteDescriptorBvhKHR = vk::WriteDescriptorSetAccelerationStructureKHR;

struct AccessFlagBits {
    using Type                         = vk::AccessFlagBits;
    static constexpr Type eBvhWriteKHR = Type::eAccelerationStructureWriteKHR;
    static constexpr Type eBvhReadKHR  = Type::eAccelerationStructureReadKHR;
};

struct PipelineStageFlagBits {
    using Type                         = vk::PipelineStageFlagBits;
    static constexpr Type eBvhBuildKHR = Type::eAccelerationStructureBuildKHR;
    static constexpr Type eRtShaderKHR = Type::eRayTracingShaderKHR;
};

struct QueryType {
    using Type                                 = vk::QueryType;
    static constexpr Type eBvhCompactedSizeKHR = Type::eAccelerationStructureCompactedSizeKHR;
    static constexpr Type eBvhSerializationSizeKHR =
        Type::eAccelerationStructureSerializationSizeKHR;
};

struct BufferUsageFlagBits {
    using Type                           = vk::BufferUsageFlagBits;
    static constexpr Type eBvhStorageKHR = Type::eAccelerationStructureStorageKHR;
    static constexpr Type eBvhBuildInputReadOnly =
        Type::eAccelerationStructureBuildInputReadOnlyKHR;
    static constexpr Type eSbtKHR = Type::eShaderBindingTableKHR;
};
}  // namespace vkrt

#else
#error This include requires VK_KHR_ray_tracing support in the Vulkan SDK.
#endif
