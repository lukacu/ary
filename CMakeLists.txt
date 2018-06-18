PROJECT(ARy)
CMAKE_MINIMUM_REQUIRED(VERSION 3.1 FATAL_ERROR)

INCLUDE(GNUInstallDirs)
INCLUDE(CMakePackageConfigHelpers)

SET(CMAKE_CXX_STANDARD 11)
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

SET(PROJECT_VERSION 0.1.0)
SET(LIB_INSTALL_DIR ${CMAKE_INSTALL_LIBDIR})
SET(INCLUDE_INSTALL_DIR ${CMAKE_INSTALL_INCLUDEDIR})

FIND_PACKAGE(OpenCV REQUIRED core imgproc video features2d calib3d)
INCLUDE_DIRECTORIES(AFTER "${OpenCV_INCLUDE_DIRS}" "include")
LINK_LIBRARIES(${OpenCV_LIBS})

SET(SOURCES
	src/camera.cpp
	src/localizer.cpp
	src/keypoints.cpp
	src/binary.cpp
	src/tracker.cpp
	src/utilities.cpp
	src/complex.cpp)

ADD_LIBRARY(ary SHARED ${SOURCES})

set_target_properties(ary PROPERTIES
    VERSION 0.0.0
    SOVERSION 0
)

ADD_LIBRARY(ary_static STATIC ${SOURCES})
SET_PROPERTY(TARGET ary_static PROPERTY POSITION_INDEPENDENT_CODE ON)

option(BUILD_DEMOS "Build ARy demos" OFF)
option(BUILD_TOOLS "Build command line tools" OFF)
option(BUILD_GODOT "Build Godot game engine API" OFF)

IF (BUILD_DEMOS)
ADD_SUBDIRECTORY(demos)
ENDIF (BUILD_DEMOS)

IF (BUILD_TOOLS)
ADD_SUBDIRECTORY(tools)
ENDIF (BUILD_TOOLS)

IF (BUILD_GODOT)
ADD_SUBDIRECTORY(api/godot)
ENDIF (BUILD_GODOT)

INSTALL(TARGETS ary EXPORT ary_targets DESTINATION ${CMAKE_INSTALL_LIBDIR})
INSTALL(FILES
	include/ary/binary.h
	include/ary/camera.h
	include/ary/complex.h
	include/ary/keypoints.h
	include/ary/localizer.h
	include/ary/tracker.h
	include/ary/utilities.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ary)

configure_package_config_file(AryConfig.cmake.in
    ${PROJECT_BINARY_DIR}/AryConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/Ary
    PATH_VARS LIB_INSTALL_DIR INCLUDE_INSTALL_DIR)

write_basic_package_version_file(
    ${PROJECT_BINARY_DIR}/AryConfig-version.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion)

INSTALL(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/AryConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/AryConfig-version.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/Ary)

INSTALL(
    EXPORT ary_targets
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/Ary
    FILE Ary-targets.cmake
)

