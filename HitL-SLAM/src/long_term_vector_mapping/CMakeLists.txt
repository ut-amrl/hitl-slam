SET(target LTVM)
ROSBUILD_ADD_EXECUTABLE(${target}
   LTVM_main.cpp
   LTVM.cpp
   ../perception_tools/kdtree.cpp)

ADD_DEPENDENCIES(${target}
   shared_libraries
   perception_library
   rosbuild_precompile
   ceres)

TARGET_LINK_LIBRARIES(${target}
   shared_libraries
   perception_library
   ${libs}
   ceres
   gflags
   gui_libraries)

