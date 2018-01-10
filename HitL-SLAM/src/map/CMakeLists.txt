#SET(target build_object_map)
#ROSBUILD_ADD_EXECUTABLE(${target}
#    build_object_map_main.cpp
#    object_map.cpp
#    ../perception_tools/kdtree.cpp
#    ../libraries/maximal_cliques/maximalcliquegenerator.cpp
#    ../libraries/connected_components/connected_components.cpp)
#ADD_DEPENDENCIES(${target}
#    shared_libraries 
#    map_libraries 
#    rosbuild_precompile 
#    ceres)
#TARGET_LINK_LIBRARIES(${target} 
#    shared_libraries 
#    map_libraries 
#    ${libs}
#    ceres 
#    gflags 
#    gui_libraries)

# SET(target convert_map)
# ROSBUILD_ADD_EXECUTABLE(${target} src/map/convert_map.cpp)
# ADD_DEPENDENCIES(${target} shared_libraries map_libraries)
# TARGET_LINK_LIBRARIES(${target} shared_libraries map_libraries ${libs})


