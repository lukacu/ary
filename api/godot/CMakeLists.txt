
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

ADD_LIBRARY(ary_godot SHARED camera.cpp wrapper.c ary.cpp)

TARGET_LINK_LIBRARIES(ary_godot ary_static ${OpenCV_LIBS})

IF(NOT EXISTS "${CMAKE_CURRENT_BINARY_DIR}/godot_headers-master")
    FILE(DOWNLOAD "https://github.com/GodotNativeTools/godot_headers/archive/master.zip" ${CMAKE_CURRENT_BINARY_DIR}/headers.zip)
    EXECUTE_PROCESS(COMMAND ${CMAKE_COMMAND} -E tar xzf ${CMAKE_CURRENT_BINARY_DIR}/headers.zip WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
endif()

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR}/godot_headers-master)

SET_TARGET_PROPERTIES(ary_godot PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/godot")



