# DisplayOpticalFlow_px4flow
Using functions from OpenCV and Ros to display optical flow from px4flow camera


#Add to CMakeLists.txt
include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(DisplayFlowImage src/DisplayFlowImage.cpp)
target_link_libraries(DisplayFlowImage ${catkin_LIBRARIES} boost_system boost_iostreams util opencv_core opencv_imgproc opencv_highgui)
add_dependencies(DisplayFlowImage DisplayFlowImage_generate_messages_cpp)
