<<<<<<< HEAD
cmake_minimum_required(VERSION 2.8)

get_filename_component(PROJNAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
SET(PROJNAME vk_${PROJNAME}_KHR)
set(CMAKE_CXX_STANDARD 17)

Message(STATUS "C++ STANDARD = ${CMAKE_CXX_STANDARD}")

Project(${PROJNAME})
Message(STATUS "-------------------------------")
Message(STATUS "Processing Project ${PROJNAME}:")

#####################################################################################
_add_project_definitions(${PROJNAME})

#####################################################################################
# Source files for this project
#
file(GLOB SOURCE_FILES *.cpp *.hpp *.inl *.h *.c)
file(GLOB EXTRA_COMMON "../common/*.*")
file(GLOB EXTRA_PBR_PARSER "../parser/*.hpp" "../parser/*.h" "../parser/*.cpp")
list(APPEND COMMON_SOURCE_FILES ${EXTRA_COMMON} ${EXTRA_PBR_PARSER})
include_directories("../common" "/usr/local/include/")

#####################################################################################
# GLSL to SPIR-V custom build
#
# more than one file can be given: _compile_GLSL("GLSL_mesh.vert;GLSL_mesh.frag" "GLSL_mesh.spv" GLSL_SOURCES)
# the SpirV validator is fine as long as files are for different pipeline stages (entry points still need to be main())
#_compile_GLSL(<source(s)> <target spv> <LIST where files are appended>)
=======
#*****************************************************************************
# Copyright 2020 NVIDIA Corporation. All rights reserved.
#*****************************************************************************

cmake_minimum_required(VERSION 3.9.6 FATAL_ERROR)

#--------------------------------------------------------------------------------------------------
# Project setting
get_filename_component(PROJNAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
SET(PROJNAME vk_${PROJNAME}_KHR)
project(${PROJNAME} LANGUAGES C CXX)
message(STATUS "-------------------------------")
message(STATUS "Processing Project ${PROJNAME}:")


#--------------------------------------------------------------------------------------------------
# C++ target and defines
set(CMAKE_CXX_STANDARD 17)
add_executable(${PROJNAME})
_add_project_definitions(${PROJNAME})


#--------------------------------------------------------------------------------------------------
# Source files for this project
#
file(GLOB SOURCE_FILES *.cpp *.hpp *.inl *.h *.c)
file(GLOB EXTRA_COMMON ${TUTO_KHR_DIR}/common/*.*)
list(APPEND COMMON_SOURCE_FILES ${EXTRA_COMMON})
include_directories(${TUTO_KHR_DIR}/common)


#--------------------------------------------------------------------------------------------------
# GLSL to SPIR-V custom build
#
SET(VULKAN_TARGET_ENV vulkan1.2)
>>>>>>> b7ba7d4fdc4128cefb7ab1d6097cc6d21c37d817
UNSET(GLSL_SOURCES)
UNSET(SPV_OUTPUT)
file(GLOB_RECURSE GLSL_HEADER_FILES "shaders/*.h" "shaders/*.glsl")
file(GLOB_RECURSE GLSL_SOURCE_FILES
    "shaders/*.comp"
    "shaders/*.frag"
    "shaders/*.vert"
    "shaders/*.rchit"
    "shaders/*.rahit"
    "shaders/*.rint"
    "shaders/*.rmiss"
    "shaders/*.rgen"
<<<<<<< HEAD
    )
#####################################################################################
# Macro to add custom build for Spir-V
# _SOURCE can be more than one file (.vert + .frag)
# _OUTPUT is the .spv file, resulting from the linkage
#
macro(L_compile_GLSL _SOURCE _OUTPUT SOURCE_LIST OUTPUT_LIST)
  if(NOT DEFINED VULKAN_TARGET_ENV)
    SET(VULKAN_TARGET_ENV vulkan1.1)
  endif()
  LIST(APPEND ${SOURCE_LIST} ${_SOURCE})
  LIST(APPEND ${OUTPUT_LIST} ${_OUTPUT})
  if(GLSLANGVALIDATOR)
    add_custom_command(
      OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/${_OUTPUT}
      COMMAND echo ${GLSLANGVALIDATOR} --target-env ${VULKAN_TARGET_ENV} -o ${_OUTPUT}  ${_SOURCE}
      COMMAND ${GLSLANGVALIDATOR} --target-env ${VULKAN_TARGET_ENV} -o ${_OUTPUT} ${_SOURCE}
      MAIN_DEPENDENCY ${_SOURCE}
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
      )
  else(GLSLANGVALIDATOR)
    MESSAGE(WARNING "could not find GLSLANGVALIDATOR to compile shaders")
  endif(GLSLANGVALIDATOR)
endmacro()
foreach(GLSL ${GLSL_SOURCE_FILES})
    get_filename_component(FILE_NAME ${GLSL} NAME)
    L_compile_GLSL(${GLSL} "shaders/${FILE_NAME}.spv" GLSL_SOURCES SPV_OUTPUT)
endforeach(GLSL)

list(APPEND GLSL_SOURCES ${GLSL_HEADER_FILES})
source_group(Shader_Files FILES ${GLSL_SOURCES})


#####################################################################################
# Executable
#
# if(WIN32 AND NOT GLUT_FOUND)
#   add_definitions(/wd4996) #remove printf warning
#   add_definitions(/wd4244) #remove double to float conversion warning
#   add_definitions(/wd4305) #remove double to float truncation warning
# else()
#   add_definitions(-fpermissive)
# endif()
add_executable(${PROJNAME} ${SOURCE_FILES} ${COMMON_SOURCE_FILES} ${PACKAGE_SOURCE_FILES} ${GLSL_SOURCES} ${CUDA_FILES} ${CUBIN_SOURCES})

#_set_subsystem_console(${PROJNAME})

#####################################################################################
# common source code needed for this sample
#
source_group(common FILES 
  ${COMMON_SOURCE_FILES}
  ${PACKAGE_SOURCE_FILES}
)
source_group("Source Files" FILES ${SOURCE_FILES})

# if(UNIX)
#   set(UNIXLINKLIBS dl pthread)
# else()
#   set(UNIXLINKLIBS)
# endif()

#####################################################################################
=======
    "shaders/*.rcall"
    )
foreach(GLSL ${GLSL_SOURCE_FILES})
    get_filename_component(FILE_NAME ${GLSL} NAME)
    _compile_GLSL(${GLSL} "shaders/${FILE_NAME}.spv" GLSL_SOURCES SPV_OUTPUT)
endforeach(GLSL)
list(APPEND GLSL_SOURCES ${GLSL_HEADER_FILES})


#--------------------------------------------------------------------------------------------------
# Sources
target_sources(${PROJNAME} PUBLIC ${SOURCE_FILES} ${HEADER_FILES})
target_sources(${PROJNAME} PUBLIC ${COMMON_SOURCE_FILES})
target_sources(${PROJNAME} PUBLIC ${PACKAGE_SOURCE_FILES})
target_sources(${PROJNAME} PUBLIC ${GLSL_SOURCES})


#--------------------------------------------------------------------------------------------------
# Sub-folders in Visual Studio
#
source_group("Common"       FILES ${COMMON_SOURCE_FILES} ${PACKAGE_SOURCE_FILES})
source_group("Sources"      FILES ${SOURCE_FILES})
source_group("Headers"      FILES ${HEADER_FILES})
source_group("Shader_Files" FILES ${GLSL_SOURCES})


#--------------------------------------------------------------------------------------------------
>>>>>>> b7ba7d4fdc4128cefb7ab1d6097cc6d21c37d817
# Linkage
#
target_link_libraries(${PROJNAME} ${PLATFORM_LIBRARIES} shared_sources)

foreach(DEBUGLIB ${LIBRARIES_DEBUG})
  target_link_libraries(${PROJNAME} debug ${DEBUGLIB})
endforeach(DEBUGLIB)

foreach(RELEASELIB ${LIBRARIES_OPTIMIZED})
  target_link_libraries(${PROJNAME} optimized ${RELEASELIB})
endforeach(RELEASELIB)

<<<<<<< HEAD

#####################################################################################
=======
#--------------------------------------------------------------------------------------------------
>>>>>>> b7ba7d4fdc4128cefb7ab1d6097cc6d21c37d817
# copies binaries that need to be put next to the exe files (ZLib, etc.)
#
_copy_binaries_to_target( ${PROJNAME} )


<<<<<<< HEAD
install(FILES ${SPV_OUTPUT} CONFIGURATIONS Release DESTINATION "bin_${ARCH}/${PROJNAME}/shaders")
install(FILES ${SPV_OUTPUT} CONFIGURATIONS Debug DESTINATION "bin_${ARCH}_debug/${PROJNAME}/shaders")
install(FILES ${CUBIN_SOURCES} CONFIGURATIONS Release DESTINATION "bin_${ARCH}/${PROJNAME}")
install(FILES ${CUBIN_SOURCES} CONFIGURATIONS Debug DESTINATION "bin_${ARCH}_debug/${PROJNAME}")
install(DIRECTORY "../media" CONFIGURATIONS Release DESTINATION "bin_${ARCH}/${PROJNAME}")
install(DIRECTORY "../media" CONFIGURATIONS Debug DESTINATION "bin_${ARCH}_debug/${PROJNAME}")
=======
#install(FILES ${SPV_OUTPUT} CONFIGURATIONS Release DESTINATION "bin_${ARCH}/${PROJNAME}/shaders")
#install(FILES ${SPV_OUTPUT} CONFIGURATIONS Debug DESTINATION "bin_${ARCH}_debug/${PROJNAME}/shaders")
#install(FILES ${CUBIN_SOURCES} CONFIGURATIONS Release DESTINATION "bin_${ARCH}/${PROJNAME}")
#install(FILES ${CUBIN_SOURCES} CONFIGURATIONS Debug DESTINATION "bin_${ARCH}_debug/${PROJNAME}")
#install(DIRECTORY "../media" CONFIGURATIONS Release DESTINATION "bin_${ARCH}/${PROJNAME}")
#install(DIRECTORY "../media" CONFIGURATIONS Debug DESTINATION "bin_${ARCH}_debug/${PROJNAME}")

#----------------------------------------------------------------------------------------------------
# Copying elements
# Media
# target_copy_to_output_dir(TARGET ${PROJECT_NAME} FILES "${TUTO_KHR_DIR}/media")
# Spir-V Shaders
target_copy_to_output_dir(
  TARGET ${PROJECT_NAME} 
  RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
  DEST_SUBFOLDER "${PROJECT_NAME}/"
  FILES ${SPV_OUTPUT}
  )

>>>>>>> b7ba7d4fdc4128cefb7ab1d6097cc6d21c37d817
