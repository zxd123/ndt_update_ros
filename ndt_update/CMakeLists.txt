aux_source_directory(. DIR_LIB_SRCS)
#set(NDT_UPDATE_SRC  ndt_update.hpp ndt_update.h ndt_update.cpp voxel_grid_covariance_update.hpp voxel_grid_covariance_update.h voxel_grid_covariance_update.cpp)                                       #设置使用的文件
add_library(ndt_lib SHARED ${DIR_LIB_SRCS})

