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
const VkAccelerationStructureKHR tlas = m.rtBuilder.getAccelerationStructure()
~~~~
*/

#include <mutex>
#include <vulkan/vulkan.h>

//#define NVVK_ALLOC_DEDICATED
//#include <nvvk/allocator_vk.hpp>
//#include <nvvk/commands_vk.hpp>
#include <nvvk/debug_util_vk.hpp>
//#include "nvmath/nvmath.h"
#include <glm/glm.hpp>
#include "vk_memory.h"

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
        std::vector<VkAccelerationStructureGeometryKHR>       asGeometry;
        std::vector<VkAccelerationStructureBuildRangeInfoKHR> asBuildOffsetInfo;
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
        VkBuildAccelerationStructureFlagsKHR flags = 0;

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

    void setup(const VkDevice& device, UniqueMemoryAllocator* allocator, uint32_t queueIndex);

    // This is an instance of a BLAS
    struct Instance {
        uint32_t                   blasId{0};      // Index of the BLAS in m_blas
        uint32_t                   instanceId{0};  // Instance Index (gl_InstanceID)
        uint32_t                   hitGroupId{0};  // Hit group index in the SBT
        uint32_t                   mask{0xFF};     // Visibility mask, will be AND-ed with ray mask
        VkGeometryInstanceFlagsKHR flags{VK_GEOMETRY_INSTANCE_TRIANGLE_FACING_CULL_DISABLE_BIT_KHR};
        glm::mat4                  transform{1.0f};  // Identity
    };

    //--------------------------------------------------------------------------------------------------
    // Destroying all allocations
    //

    void destroy();

    // Returning the constructed top-level acceleration structure
    VkAccelerationStructureKHR getAccelerationStructure() const { return m_tlas.as.handle; }

    //--------------------------------------------------------------------------------------------------
    // Create all the BLAS from the vector of BlasInput
    // - There will be one BLAS per input-vector entry
    // - There will be as many BLAS as input.size()
    // - The resulting BLAS (along with the inputs used to build) are stored in m_blas,
    //   and can be referenced by index.

    void buildBlas(const std::vector<RaytracingBuilderKHR::BlasInput>& input,
                   VkBuildAccelerationStructureFlagsKHR                flags =
                       VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR);


    //--------------------------------------------------------------------------------------------------
    // Convert an Instance object into a VkAccelerationStructureInstanceKHR

    VkAccelerationStructureInstanceKHR instanceToVkGeometryInstanceKHR(const Instance& instance);

    //--------------------------------------------------------------------------------------------------
    // Creating the top-level acceleration structure from the vector of Instance
    // - See struct of Instance
    // - The resulting TLAS will be stored in m_tlas
    // - update is to rebuild the Tlas with updated matrices
    void buildTlas(const std::vector<Instance>&         instances,
                   VkBuildAccelerationStructureFlagsKHR flags =
                       VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR,
                   bool update = false);

    //--------------------------------------------------------------------------------------------------
    // Refit BLAS number blasIdx from updated buffer contents.
    //
    void updateBlas(uint32_t blasIdx);

  private:
    // Top-level acceleration structure
    struct Tlas {
        UniqueMemoryAccelStruct                       as;
        VkBuildAccelerationStructureFlagsKHR flags = 0;
    };

    //--------------------------------------------------------------------------------------------------
    // Vector containing all the BLASes built in buildBlas (and referenced by the TLAS)
    std::vector<BlasEntry> m_blas;
    // Top-level acceleration structure
    Tlas m_tlas;
    // Instance buffer containing the matrices and BLAS ids
    vkpbr::UniqueMemoryBuffer m_instBuffer;

    VkDevice m_device{VK_NULL_HANDLE};
    uint32_t m_queueIndex{0};

    UniqueMemoryAllocator* m_alloc = nullptr;
    nvvk::DebugUtil  m_debug;

#ifdef VULKAN_HPP
  public:
    void vkpbr::RaytracingBuilderKHR::buildBlas(
        const std::vector<RaytracingBuilderKHR::BlasInput>& blas_,
        vk::BuildAccelerationStructureFlagsKHR              flags)
    {
        buildBlas(blas_, static_cast<VkBuildAccelerationStructureFlagsKHR>(flags));
    }

    void vkpbr::RaytracingBuilderKHR::buildTlas(const std::vector<Instance>&           instances,
                                               vk::BuildAccelerationStructureFlagsKHR flags,
                                               bool update = false)
    {
        buildTlas(instances, static_cast<VkBuildAccelerationStructureFlagsKHR>(flags), update);
    }

#endif
};

}  // namespace vkpbr

#else
#error This include requires VK_KHR_ray_tracing support in the Vulkan SDK.
#endif
