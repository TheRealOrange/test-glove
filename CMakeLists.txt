cmake_minimum_required(VERSION 3.20)
project(test_glove)

set(CMAKE_CXX_STANDARD 17)

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

add_executable(test src/test.cpp src/GloveData.cpp)
target_include_directories(test PUBLIC includes/)

if (UNIX)
    target_link_directories(test_glove PUBLIC SenseGloveAPI/lib/linux/Release)
    target_link_directories(test PUBLIC SenseGloveAPI/lib/linux/Release)
elseif (WIN32)
    target_link_directories(test_glove PUBLIC SenseGloveAPI/lib/win/Release)
    target_link_directories(test PUBLIC SenseGloveAPI/lib/linux/Release)
endif()

target_include_directories(test_glove PUBLIC SenseGloveAPI/incl)
target_include_directories(test PUBLIC SenseGloveAPI/incl)
target_link_libraries(test_glove SGCoreCpp)
target_link_libraries(test SGCoreCpp)

set_property(TARGET test_glove PROPERTY
        MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")



