

ADD_EXECUTABLE(ary_demo_keypoints demo_keypoints.cpp)
TARGET_LINK_LIBRARIES(ary_demo_keypoints ary)

ADD_EXECUTABLE(ary_demo_binary demo_binary.cpp)
TARGET_LINK_LIBRARIES(ary_demo_binary ary)

ADD_EXECUTABLE(ary_demo_tracking demo_tracking.cpp)
TARGET_LINK_LIBRARIES(ary_demo_tracking ary)

ADD_EXECUTABLE(ary_demo_scene demo_scene.cpp)
TARGET_LINK_LIBRARIES(ary_demo_scene ary)
