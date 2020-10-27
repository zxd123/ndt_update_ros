SET( CMAKE_CXX_FLAGS "-std=c++14 -O3")
set(CMAKE_CXX_STANDARD 14)
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(normal_distributions_transform)
find_package(PCL 1.11 REQUIRED)
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/ndt_update")
MESSAGE("CMAKE_CURRENT_SOURCE_DIR = "${CMAKE_CURRENT_SOURCE_DIR} )
include_directories(${PCL_INCLUDE_DIRS})
#include_directories(${PCL_INCLUDE_DIRS})

set(NDT_UPDATE_LIBRARY_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/cmake-build-debug/ndt_update")
link_directories(${PCL_LIBRARY_DIRS} ${NDT_UPDATE_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

# 查找当前目录下的所有源文件
# 并将名称保存到 DIR_SRCS 变量
aux_source_directory(. DIR_SRCS)
#ADD_SUBDIRECTORY(ndt_update)
add_executable(ndt_test ${DIR_SRCS})

#add_dependencies(ndt_test ndt_lib)

#target_link_libraries (ndt_test ${PCL_LIBRARIES} )

target_link_libraries (ndt_test ${PCL_LIBRARIES} )

install(TARGETS ndt_test RUNTIME DESTINATION bin)
