cmake_minimum_required(VERSION 3.20)
project(test_glove)
set(CMAKE_CXX_STANDARD 17)


# Make project require C++11
#include(CheckCXXCompilerFlag)
#CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
#CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
#if(COMPILER_SUPPORTS_CXX11)
#    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11")
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#elseif(COMPILER_SUPPORTS_CXX0X)
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
#endif()



set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)

include_directories("third-party/glfw/include")
include_directories("third-party/GL")
add_subdirectory("third-party/glfw")

# Simple non robust way to find the librealsense library
if(WIN32)
    if("${CMAKE_SIZEOF_VOID_P}" STREQUAL "8")
        set(LIBRARY_DIR "C:\\Program Files (x86)\\Intel RealSense SDK 2.0\\lib\\x64")          # TODO: Update this variable to correct path - folder where realsense2.lib is found
        set(DLL_DIR "C:\\Program Files (x86)\\Intel RealSense SDK 2.0\\bin\\x64")              # TODO: Update this variable to correct path - folder where realsense2.dll is found
    else()
        set(LIBRARY_DIR "C:\\Program Files (x86)\\Intel RealSense SDK 2.0\\lib\\x86")          # TODO: Update this variable to correct path - folder where realsense2.lib is found
        set(DLL_DIR "C:\\Program Files (x86)\\Intel RealSense SDK 2.0\\bin\\x86")              # TODO: Update this variable to correct path - folder where realsense2.dll is found
    endif()
    set(PROJECT_BINARY_OUTPUT_PATH "${PROJECT_BINARY_DIR}/\$\(Configuration\)")                # TODO: Update this variable to correct path - folder to which your project will compile
    set(ADDITIONAL_INCLUDE_DIRS "C:\\Program Files (x86)\\Intel RealSense SDK 2.0\\include")   # TODO: Update this variable to correct path - folder where librealsense2 folder is found
    find_library(REALSENSE2_FOUND realsense2 HINTS ${LIBRARY_DIR} REQUIRED)
    if(NOT REALSENSE2_FOUND)
        SET(REALSENSE2_FOUND "realsense2")
        message(WARN "Failed to find_library(realsense2)")
    endif()
elseif(UNIX)
    find_package(realsense2 REQUIRED)
endif()

if(DEFINED ENV{CINDER_LIB_PATH})
    cmake_path(SET CINDER_PATH "$ENV{CINDER_LIB_PATH}" )
    message(STATUS " Cinder located at: $ENV{CINDER_LIB_PATH}")
else()
    message(ERROR " environment variable CINDER_LIB_PATH not set")
endif()

if (UNIX)
    set( CINDER_TARGET "linux")
    message(STATUS " Linux detected")
elseif (WIN32)
    set( CINDER_TARGET "msw")
    message(STATUS " Windows detected")
else ()
    message(ERROR " OS not supported")
endif()

cmake_path(SET APP_PATH "${CMAKE_CURRENT_SOURCE_DIR}" )

include( "${CINDER_PATH}/proj/cmake/modules/cinderMakeApp.cmake" )
ci_make_app(
        APP_NAME    "test_glove"
        CINDER_PATH ${CINDER_PATH}
        SOURCES     ${APP_PATH}/src/DataCaptureApp.cpp ${APP_PATH}/src/GloveData.cpp
        INCLUDES    ${APP_PATH}/SenseGloveAPI/incl ${APP_PATH}/includes
        #RESOURCES   ${APP_PATH}/
)

if (UNIX)
    target_link_directories(test_glove PUBLIC SenseGloveAPI/lib/linux/Release)
elseif (WIN32)
    target_link_directories(test_glove PUBLIC SenseGloveAPI/lib/win/Release)
endif()

target_include_directories(test_glove PUBLIC SenseGloveAPI/incl)
target_link_libraries(test_glove SGCoreCpp)

set_property(TARGET test_glove PROPERTY
        MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

find_package(OpenGL REQUIRED)

# realsense executable
set(HEADER_FILES includes/RealSense.h third-party/glfw/include/GLFW/glfw3.h third-party/GL/glu.h)
add_executable(realsense src/RealSense.cpp ${HEADER_FILES})
include_directories(realsense ${ADDITIONAL_INCLUDE_DIRS})
target_link_libraries(realsense ${REALSENSE2_FOUND})
target_link_libraries(realsense glfw)
target_link_libraries(realsense OpenGL::GL OpenGL::GLU)
set_target_properties (realsense PROPERTIES FOLDER Examples)

#set(PCL_DIR "C:/Users/Mel/vcpkg/buildtrees/pcl/x86-windows-rel")
#set(Eigen3_DIR "C:/Users/Mel/vcpkg/buildtrees/eigen3/x86-windows-rel")

#set(Boost_DEBUG ON)
#set(BOOST_ROOT C://Program Files//PCL 1.12.0//3rdParty//Boost)

#set(Boost_USE_STATIC_LIBS ON)
#set(Boost_INCLUDE_DIR "C:/Program Files/PCL 1.12.0/3rdParty/Boost/include/boost-1_77")

#set(VTK_DIR "C://Program Files//PCL 1.12.0//3rdParty//VTK//lib//cmake//vtk-9.0")
#set(VTK_DIR "C://Program Files//PCL 1.12.0//3rdParty//VTK//lib//cmake")

find_package(PCL 1.3 REQUIRED COMPONENTS common io visualization filters)
message("PCL_LIBRARIES - ${PCL_LIBRARIES}")
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(FLANN REQUIRED)

set(HEADER_FILES includes/region_growing_rgb_segmentation.h)
add_executable(asdftesting src/region_growing_rgb_segmentation.cpp)

set(HEADER_FILES includes/capture_pointcloud.h)
add_executable(cappc src/capture_pointcloud.cpp)
include_directories(cappc ${ADDITIONAL_INCLUDE_DIRS})
#include_directories(cappc ${PCL_INCLUDE_DIRS})
target_link_libraries(cappc ${REALSENSE2_FOUND})
target_link_libraries(cappc ${PCL_LIBRARIES} )







