SET(target localization_gui)
ROSBUILD_ADD_EXECUTABLE(${target} 
    vector_display_main.cpp
    vector_display_thread.cpp)
ADD_DEPENDENCIES(${target}
    shared_libraries
    gui_libraries
    rosbuild_precompile)
    #map_libraries)
TARGET_LINK_LIBRARIES(${target}
    shared_libraries
    gui_libraries
    ${QT_LIBRARIES}
    ${libs})
    #map_libraries)

