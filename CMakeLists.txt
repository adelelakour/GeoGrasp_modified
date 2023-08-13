cmake_minimum_required(VERSION 3.1.0)
project(geograsp)


find_package(PCL REQUIRED)
find_package(realsense2 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(glfw3 3.3 REQUIRED)


include_directories(
        ${CMAKE_SOURCE_DIR}/include/
        ${PCL_INCLUDE_DIRS}
        ${VTK_INCLUDE_DIRS}
        /usr/local/include/vtk-9.2/
        /include/geograsp/utility_functions.h
        ${OPENGL_INCLUDE_DIR}
        ${GLFW_INCLUDE_DIRS}

)


set(SOURCES
        src/CompareClouds_tuple.cpp
        )
add_executable(cloud_processor ${SOURCES})


if(NOT glfw3_DIR)
    target_include_directories(cloud_processor PUBLIC "${CMAKE_SOURCE_DIR}/librealsense-2.54.1/third-party/glfw/include" ${PCL_INCLUDE_DIRS})
    target_link_directories(cloud_processor PUBLIC "${CMAKE_BINARY_DIR}/librealsense-2.54.1/third-party/glfw/src/${CMAKE_BUILD_TYPE}")
endif()


add_library(GeoGrasp
        lib/${PROJECT_NAME}/GeoGrasp.cpp
        )



target_link_libraries(cloud_processor
        ${DEPENDENCIES}
        ${realsense2_LIBRARY}
        ${VTK_LIBRARIES}
        GeoGrasp
        ${PCL_LIBRARIES}
        ${OPENGL_LIBRARIES}
        glfw

        )

foreach(flag_var
        CMAKE_CXX_FLAGS CMAKE_CXX_FLAGS_DEBUG CMAKE_CXX_FLAGS_RELEASE
        CMAKE_CXX_FLAGS_MINSIZEREL CMAKE_CXX_FLAGS_RELWITHDEBINFO
        CMAKE_C_FLAGS CMAKE_C_FLAGS_DEBUG CMAKE_C_FLAGS_RELEASE
        CMAKE_C_FLAGS_MINSIZEREL CMAKE_C_FLAGS_RELWITHDEBINFO)
    if(${flag_var} MATCHES "/MT")
        string(REGEX REPLACE "/MT" "/MD" ${flag_var} "${${flag_var}}")
    endif(${flag_var} MATCHES "/MT")
endforeach(flag_var)



#install(DIRECTORY include/${PROJECT_NAME}/
# DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#FILES_MATCHING PATTERN "*.h"
#PATTERN ".svn" EXCLUDE)






#target_include_directories(cloud_processor PRIVATE ${EIGEN3_INCLUDE_DIR})


#set_property(TARGET cloud_processor PROPERTY CXX_STANDARD 14)




