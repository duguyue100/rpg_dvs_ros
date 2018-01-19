cmake_minimum_required(VERSION 2.8.3)
project(dvs_binder)

find_package(catkin_simple REQUIRED)

catkin_simple()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

find_package(OpenCV REQUIRED)

# make the executable
cs_add_executable(dvs_binder
  src/image_tracking.cpp
  src/binder.cpp
  src/binder_node.cpp
)

# make the nodelet into a library
cs_add_library(dvs_binder_nodelet
  src/image_tracking.cpp
  src/binder_nodelet.cpp
  src/binder.cpp
)

# link the executable to the necesarry libs
target_link_libraries(dvs_binder
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)

target_link_libraries(dvs_binder_nodelet
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)

cs_install()

# Install other support files for installation
install(FILES dvs_binder_nodelet.xml davis_mono.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
