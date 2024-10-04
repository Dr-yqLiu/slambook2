cmake_minimum_required(VERSION 2.8)
project(useEigen)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-O3")

# 添加Eigen头文件
# include_directories("/usr/include/eigen3")
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(eigenMatrix eigenMatrix.cpp)
target_link_libraries(eigenMatrix Eigen3::Eigen)
