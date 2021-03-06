cmake_minimum_required(VERSION 3.8)
message("@@@@@@@@@@@@@@ MARS cmake list @@@@@@@@@@@@@@")

set(CUDA_SDK_ROOT_DIR "/usr/local/cuda/samples")



if(NOT DEFINED CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
  set(CMAKE_CXX_STANDARD_REQUIRED True)
endif()

if(NOT DEFINED CMAKE_CUDA_STANDARD)
  set(CMAKE_CUDA_STANDARD 11)
  set(CMAKE_CUDA_STANDARD_REQUIRED True)
endif()

# set(BINARY_INSTALL_PATH "bin" CACHE PATH "Binary install path")
# set(LIBRARY_INSTALL_PATH "lib" CACHE PATH "Library install path")
# set(INCLUDE_INSTALL_PATH "include" CACHE PATH "Include install path")
# set(CONFIG_INSTALL_PATH "${LIBRARY_INSTALL_PATH}/MARS/cmake")

# set(DESPOT_BUILD_EXAMPLES ON CACHE BOOL "Build C++ model examples")
# set(DESPOT_BUILD_POMDPX ON CACHE BOOL "Build POMDPX example")
# set(DESPOT_BUILD_GPUS ON CACHE BOOL "Build GPU despot")

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse2 -mfpmath=sse")
# set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -arch=sm_30")
# set(CMAKE_MODULE_PATH ${CMAKE_PREFIX_PATH} "${PROJECT_SOURCE_DIR}/cmake")


if (CUDA)
endif()


find_package(catkin REQUIRED COMPONENTS
  roscpp
)


include_directories(
   ${catkin_INCLUDE_DIRS}
   ../../HypDespot/include/
   ./
   src/GPU_MA_RS/
   src/ma_rock_sample/
   src/base/
)
message("@@@@@@@@@@@@@@ before add lib @@@@@@@@@@@@@@")

add_executable(${PROJECT_NAME}_mars
  src/ma_rock_sample/main.cu
  src/ma_rock_sample/ma_rock_sample.cpp
  src/base/base_ma_rock_sample.cpp
  src/GPU_MA_RS/GPU_base_ma_rock_sample.cu  
  src/GPU_MA_RS/GPU_ma_rock_sample.cu

)
message("@@@@@@@@@@@@@@ after add lib @@@@@@@@@@@@@@")

target_link_libraries(${PROJECT_NAME}_mars
   ${catkin_LIBRARIES}
   ${PROJECT_NAME}
)

set_target_properties(
			${PROJECT_NAME}_mars
                       PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
set_target_properties(
			${PROJECT_NAME}_mars
                       PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_compile_definitions(
  ${PROJECT_NAME}_mars  
  PRIVATE KW_EXPORTS
  INTERFACE KW_IMPORTS
)

target_link_libraries(${PROJECT_NAME}_mars
  ${TinyXML_LIBRARIES}
)


# Install a DespotConfig.cmake file so CMake can find_package(Despot).
include(CMakePackageConfigHelpers)
configure_package_config_file("cmake/DespotConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/DespotConfig.cmake"
  INSTALL_DESTINATION "${CONFIG_INSTALL_PATH}"
  PATH_VARS INCLUDE_INSTALL_PATH
)


message("@@@@@@@@@@@@@@ end @@@@@@@@@@@@@@")

