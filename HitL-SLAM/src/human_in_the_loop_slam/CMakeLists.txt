SET(target HitL_SLAM)
ROSBUILD_ADD_EXECUTABLE(${target}
   HitLSLAM_main.cpp
   EMinput.cpp
   ApplyExplicitCorrection.cpp
   Backprop.cpp
   JointOptimization.cpp
   ../vmapping/vector_mapping.cpp
   ../perception_tools/kdtree.cpp)

ADD_DEPENDENCIES(${target}
   shared_libraries
   perception_library
   rosbuild_precompile
   ceres)
   #pf_libraries)
TARGET_LINK_LIBRARIES(${target}
   shared_libraries
   perception_library
   ${libs}
   ceres
   gflags
   gui_libraries)

