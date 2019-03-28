cmake_minimum_required(VERSION 3.8)

message("@@@@@@@@@@@@@@ Grasping Ros Mico cmake list @@@@@@@@@@@@@@")
set(CUDA_SDK_ROOT_DIR "/usr/local/cuda/samples")

#project(grasping_ros_mico CXX CUDA)
#find_package(CUDA REQUIRED)

if(NOT DEFINED CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
  set(CMAKE_CXX_STANDARD_REQUIRED True)
endif()

if(NOT DEFINED CMAKE_CUDA_STANDARD)
  set(CMAKE_CUDA_STANDARD 11)
  set(CMAKE_CUDA_STANDARD_REQUIRED True)
endif()

find_package(TensorFlow REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_GLIBCXX_USE_CXX11_ABI=${TensorFlow_ABI}")
set(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} -D_GLIBCXX_USE_CXX11_ABI=${TensorFlow_ABI}" )

TensorFlow_REQUIRE_C_LIBRARY()
TensorFlow_REQUIRE_SOURCE()
#set(CMAKE_PREFIX_PATH "/opt/ros/kinetic")

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse2 -mfpmath=sse")
#set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -arch=sm_30")




#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#set(DESPOT_HOME "../despot/")
#set(HYPDESPOTALPHA_HOME "../hyp-despot-alpha/")

message("@@@@@@@@@@@@@@@@@@@@@2 ${TensorFlow_INCLUDE_DIR} @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")

#create despot lib
# search header files in those sub directories
# Maybe a glob could be possible
INCLUDE_DIRECTORIES(
${catkin_INCLUDE_DIRS}
	#src/despot/
        src/Display
        ${TensorFlow_INCLUDE_DIR}
        ${TensorFlow_SOURCE_DIR}
        #msg_gen/cpp/include/despot_vrep_glue
)
set(WARNING_FLAGS -Wall -Wextra -Wno-sign-compare)
set(PED_COMPILE_FLAGS -std=c++0x -O3 -c ${WARNING_FLAGS})


set(CUDA_SEPARABLE_COMPILATION ON)
set(POSITION_INDEPENDENT_CODE ON)

#FILE(GLOB LIBDESPOT_SRCS src/despot/*.cpp src/despot/util/*.cpp src/despot/util/pomdpx_parser/*.cpp src/despot/problems/*.cpp)
#rosbuild_add_library(despot ${LIBDESPOT_SRCS})
#rosbuild_add_compile_flags(despot ${PED_COMPILE_FLAGS})

include_directories(
  "../../HypDespot/include/"
#"${HYPDESPOTALPHA_HOME}/src/HypDespot/include"
"src/despot_ext/include"
#"../graspingV4/src/"
)

#find_library(DESPOT_LIB NAMES despot HINTS "${DESPOT_HOME}/build")
#find_library(HYPDESPOTALPHA_LIB NAMES hyp_despot PATHS "${HYPDESPOTALPHA_HOME}/Release/devel/lib" HINTS "${HYPDESPOTALPHA_HOME}/Release")
#find_library(DESPOT_EXT_LIB NAMES despot_ext HINTS "../build/despot_ext")

 find_package( Boost COMPONENTS system REQUIRED )
 link_directories( ${Boost_LIBRARY_DIRS} )
 include_directories( ${Boost_INCLUDE_DIRS} "/usr/include/python2.7")


find_package(PythonLibs 2.7 REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

find_library(YAML-CPP yaml-cpp /usr/local/lib)
FILE(GLOB EXEC_SRCS src/*.cpp src/Display/c*.cpp src/despot_ext/src/*.cpp)
set(WITHOUT_DISPLAY_SRCS ${EXEC_SRCS})
set(GATHER_DATA_SRCS ${EXEC_SRCS})
list(REMOVE_ITEM EXEC_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/src/main_gather_data.cpp")
list(REMOVE_ITEM EXEC_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/src/main_without_display.cpp")
list(REMOVE_ITEM WITHOUT_DISPLAY_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/src/main_gather_data.cpp")
list(REMOVE_ITEM WITHOUT_DISPLAY_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp")
list(REMOVE_ITEM GATHER_DATA_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp")
list(REMOVE_ITEM GATHER_DATA_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/src/main_without_display.cpp")

add_executable(gather_data ${GATHER_DATA_SRCS})
add_executable(grasping_ros_mico ${EXEC_SRCS})
add_executable(despot_without_display ${WITHOUT_DISPLAY_SRCS})
add_executable (inference_cc src/inference_cc.cc)
target_include_directories(inference_cc PRIVATE TensorFlow_DEP)
target_link_libraries(inference_cc PRIVATE TensorFlow_DEP)


add_dependencies(grasping_ros_mico ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(despot_without_display ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(gather_data ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



target_link_libraries(grasping_ros_mico ${catkin_LIBRARIES})
target_link_libraries(grasping_ros_mico ${PROJECT_NAME} )
#target_link_libraries(grasping_ros_mico ${DESPOT_EXT_LIB})
target_link_libraries(grasping_ros_mico ${YAML-CPP})
target_link_libraries( grasping_ros_mico ${Boost_SYSTEM_LIBRARY} )
target_link_libraries(grasping_ros_mico ${PYTHON_LIBRARIES})

target_link_libraries(despot_without_display ${catkin_LIBRARIES})
target_link_libraries(despot_without_display ${PROJECT_NAME})
#target_link_libraries(despot_without_display ${DESPOT_EXT_LIB})
target_link_libraries(despot_without_display ${YAML-CPP})
target_link_libraries(despot_without_display ${Boost_SYSTEM_LIBRARY} )
target_link_libraries(despot_without_display ${PYTHON_LIBRARIES})
#target_include_directories(despot_without_display PRIVATE TensorFlow_DEP)
#target_include_directories(despot_without_display SYSTEM INTERFACE ${TensorFlow_INCLUDE_DIR})
TARGET_LINK_LIBRARIES(despot_without_display ${TensorFlow_C_LIBRARY})
target_link_libraries(despot_without_display ${TensorFlow_LIBRARY})


target_link_libraries(gather_data ${catkin_LIBRARIES})
target_link_libraries(gather_data ${PROJECT_NAME})
#target_link_libraries(gather_data ${DESPOT_EXT_LIB})
target_link_libraries(gather_data ${YAML-CPP})
target_link_libraries( gather_data ${Boost_SYSTEM_LIBRARY} )
target_link_libraries(gather_data ${PYTHON_LIBRARIES})
#target_link_libraries(gather_data ${CUDA_LIBRARIES})
#message("Cuda libraries ${CUDA_LIBRARIES}")
#target_link_libraries(gather_data /usr/local/cuda-9.0/targets/x86_64-linux/lib/libcudart.so)

set_target_properties( gather_data
                       PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
set_target_properties( gather_data
                       PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_definitions(
  gather_data
  PRIVATE KW_EXPORTS
  INTERFACE KW_IMPORTS
)

set_target_properties( grasping_ros_mico
                       PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
set_target_properties( grasping_ros_mico
                       PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_definitions(
  grasping_ros_mico
  PRIVATE KW_EXPORTS
  INTERFACE KW_IMPORTS
)

set_target_properties( despot_without_display
                       PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
set_target_properties( despot_without_display
                       PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_definitions(
  despot_without_display
  PRIVATE KW_EXPORTS
  INTERFACE KW_IMPORTS
)


#add_compile_flags(grasping_ros_mico ${PED_COMPILE_FLAGS})
#add_compile_flags(despot_without_display ${PED_COMPILE_FLAGS})
#add_compile_flags(gather_data ${PED_COMPILE_FLAGS})

add_executable(display src/Display/mainCupDisplay.cpp src/Display/cupDisplay.cpp)
add_dependencies(display ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(display ${Boost_SYSTEM_LIBRARY} )
target_link_libraries(display ${catkin_LIBRARIES})
#rosbuild_add_compile_flags(mico_despot_planner ${PED_COMPILE_FLAGS})


## Install a DespotConfig.cmake file so CMake can find_package(Despot).
#include(CMakePackageConfigHelpers)
#configure_package_config_file("cmake/DespotConfig.cmake.in"
#  "${CMAKE_CURRENT_BINARY_DIR}/DespotConfig.cmake"
#  INSTALL_DESTINATION "${CONFIG_INSTALL_PATH}"
#  PATH_VARS INCLUDE_INSTALL_PATH
#)


message("@@@@@@@@@@@@@@ end @@@@@@@@@@@@@@")
