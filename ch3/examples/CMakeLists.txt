find_package(Eigen3 3.3 REQUIRED NO_MODULE)
add_executable(coordinateTransform coordinateTransform.cpp)
target_link_libraries(coordinateTransform Eigen3::Eigen)

find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})
add_executable(plotTrajectory plotTrajectory.cpp)
target_link_libraries(plotTrajectory ${Pangolin_LIBRARIES})
target_link_libraries(plotTrajectory Eigen3::Eigen)
