cmake_minimum_required(VERSION 3.8)
message("@@@@@@@@@@@@@@ GPUmodel cmake list @@@@@@@@@@@@@@")

set(CUDA_SDK_ROOT_DIR "/usr/local/cuda/samples")


project(hyp_despot CXX CUDA)


if(NOT DEFINED CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
  set(CMAKE_CXX_STANDARD_REQUIRED True)
endif()

if(NOT DEFINED CMAKE_CUDA_STANDARD)
  set(CMAKE_CUDA_STANDARD 11)
  set(CMAKE_CUDA_STANDARD_REQUIRED True)
endif()


set(BINARY_INSTALL_PATH "bin" CACHE PATH "Binary install path")
set(LIBRARY_INSTALL_PATH "lib" CACHE PATH "Library install path")
set(INCLUDE_INSTALL_PATH "include" CACHE PATH "Include install path")
set(CONFIG_INSTALL_PATH "${LIBRARY_INSTALL_PATH}/GPUmodel/cmake")

set(DESPOT_BUILD_EXAMPLES ON CACHE BOOL "Build C++ model examples")
set(DESPOT_BUILD_POMDPX ON CACHE BOOL "Build POMDPX example")
set(DESPOT_BUILD_GPUS ON CACHE BOOL "Build GPU despot")


set(CMAKE_PREFIX_PATH "/home/yuanfu/workspace/catkin_ws/devel;/opt/ros/kinetic")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse2 -mfpmath=sse")
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -arch=sm_30")
set(CMAKE_MODULE_PATH ${CMAKE_PREFIX_PATH} "${PROJECT_SOURCE_DIR}/cmake")




if (CUDA)
endif()


# Pass options to NVCC
set(
    CUDA_NVCC_FLAGS
    ${CUDA_NVCC_FLAGS};
    -O3 -gencode arch=compute_30,code=sm_30
    #-g -G
    )

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

#add_subdirectory(src/HypDespot/)
set(BUILD_SHARED_LIBS OFF)

set(CUDA_SEPARABLE_COMPILATION ON)

set(POSITION_INDEPENDENT_CODE ON)

include_directories(
   ${catkin_INCLUDE_DIRS}
   src/RVO2/src
   src/HypDespot/include/
)

FILE(GLOB LIBRVO_SRCS  src/RVO2/src/*.cpp)

catkin_package(
  #INCLUDE_DIRS src/planner/
#  LIBRARIES ped_path_predictor
  CATKIN_DEPENDS roscpp #std_msgs
  DEPENDS system_lib
)
message("@@@@@@@@@@@@@@ before add lib @@@@@@@@@@@@@@")
add_subdirectory(src/HypDespot/)


# add_library(
# "${PROJECT_NAME}" STATIC
#   src/HypDespot/src/interface/pomdp.cpp
#   src/HypDespot/src/interface/world.cpp
#   src/HypDespot/src/interface/belief.cpp
#   src/HypDespot/src/interface/lower_bound.cpp
#   src/HypDespot/src/interface/upper_bound.cpp
#   src/HypDespot/src/interface/default_policy.cpp
#   src/HypDespot/src/core/particle_belief.cpp
#   src/HypDespot/src/core/globals.cpp
#   src/HypDespot/src/core/builtin_lower_bounds.cpp
#   src/HypDespot/src/core/mdp.cpp
#   src/HypDespot/src/core/node.cpp
#   src/HypDespot/src/core/builtin_policy.cpp
#   src/HypDespot/src/core/pomdp_world.cpp
#   src/HypDespot/src/core/solver.cpp
#   src/HypDespot/src/core/builtin_upper_bounds.cpp
#   src/HypDespot/src/logger.cpp
#   src/HypDespot/src/planner.cpp
#   src/HypDespot/src/Parallel_planner.cu
#   src/HypDespot/src/evaluator.cpp
#   src/HypDespot/src/pomdpx/parser/function.cpp
#   src/HypDespot/src/pomdpx/parser/parser.cpp
#   src/HypDespot/src/pomdpx/parser/variable.cpp
#   src/HypDespot/src/pomdpx/pomdpx.cpp
#   src/HypDespot/src/random_streams.cpp
#   src/HypDespot/src/plannerbase.cpp
#   src/HypDespot/src/solver/aems.cpp
#   src/HypDespot/src/solver/despot.cpp
#   src/HypDespot/src/solver/pomcp.cpp
#   src/HypDespot/src/solver/baseline_solver.cpp
#   src/HypDespot/src/util/coord.cpp
#   src/HypDespot/src/util/dirichlet.cpp
#   src/HypDespot/src/util/exec_tracker.cpp
#   src/HypDespot/src/util/floor.cpp
#   src/HypDespot/src/util/gamma.cpp
#   src/HypDespot/src/util/logging.cpp
#   src/HypDespot/src/util/random.cpp
#   src/HypDespot/src/util/seeds.cpp
#   src/HypDespot/src/util/util.cpp
#   src/HypDespot/src/util/tinyxml/tinystr.cpp
#   src/HypDespot/src/util/tinyxml/tinyxml.cpp
#   src/HypDespot/src/util/tinyxml/tinyxmlerror.cpp
#   src/HypDespot/src/util/tinyxml/tinyxmlparser.cpp

#   src/HypDespot/src/GPUinterface/GPUdefault_policy.cu  
#   src/HypDespot/src/GPUinterface/GPUpomdp.cu 
#   src/HypDespot/src/GPUinterface/GPUlower_bound.cu  
#   src/HypDespot/src/GPUinterface/GPUupper_bound.cu  
#   src/HypDespot/src/GPUcore/GPUbuiltin_lower_bound.cu
#   src/HypDespot/src/GPUcore/GPUbuiltin_policy.cu
#   src/HypDespot/src/GPUcore/GPUbuiltin_upper_bound.cu
#   src/HypDespot/src/GPUcore/GPUconfig.cu   
#   src/HypDespot/src/GPUcore/GPUhistory.cu      
#   src/HypDespot/src/GPUcore/GPUnode.cu  
#   src/HypDespot/src/GPUcore/GPUglobals.cu  
#   src/HypDespot/src/GPUcore/shared_node.cu
#   src/HypDespot/src/GPUcore/thread_globals.cu
#   src/HypDespot/src/GPUutil/GPUmemorypool.cu  
#   src/HypDespot/src/GPUutil/GPUseeds.cu    
#   src/HypDespot/src/GPUutil/GPUutil.cu
#   src/HypDespot/src/GPUutil/GPUcoord.cu      
#   src/HypDespot/src/GPUutil/GPUrandom.cu      
#   src/HypDespot/src/GPUrandom_streams.cu
#   src/HypDespot/src/solver/Hyp_despot.cu
#   ${LIBRVO_SRCS}
#   # src/planner/Path.cpp
#   # src/planner/simulator.cu
#   #   src/planner/collision.cpp
#   #   src/planner/coord.cpp                  
#   #   src/planner/math_utils.cpp
#   #   src/planner/ped_pomdp.cpp
#   #   src/planner/WorldModel.cpp
#   #   src/planner/custom_particle_belief.cpp  
#   #   src/planner/param.cpp
#   #   src/planner/GPU_Car_Drive/GPU_Car_Drive.cu     
#   #   src/planner/GPU_Car_Drive/GPU_LowerBoundPolicy.cu
#   #   src/planner/GPU_Car_Drive/GPU_Init.cu         
#   #   src/planner/GPU_Car_Drive/GPU_Path.cu
#   #   src/planner/GPU_Car_Drive/GPU_CarUpperBound.cu  
#   #   src/planner/GPU_Car_Drive/GPU_param.cu  
#   #OPTIONS --compiler-options "-fPIC"
# )
# message("@@@@@@@@@@@@@@ after add lib @@@@@@@@@@@@@@")


add_subdirectory(src/HyP_examples/CarDriving)
# add_subdirectory(src/HyP_examples/ma_rock_sample)
# add_subdirectory(src/HyP_examples/unkown_navigation)

set_target_properties( "${PROJECT_NAME}"
                       PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
set_target_properties( "${PROJECT_NAME}"
                       PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_compile_definitions(${PROJECT_NAME}
  PRIVATE KW_EXPORTS
  INTERFACE KW_IMPORTS
)

target_link_libraries("${PROJECT_NAME}"
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

