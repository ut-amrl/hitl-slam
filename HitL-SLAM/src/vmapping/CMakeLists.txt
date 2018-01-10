SET(target vector_mapping)
ROSBUILD_ADD_EXECUTABLE(${target}
    vector_mapping_main.cpp
    vector_mapping.cpp
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
    #pf_libraries)

