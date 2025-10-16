cmake_minimum_required(VERSION 3.16.0 FATAL_ERROR) 
project(main VERSION 0.1.0)

add_definitions(-std=c++14)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

find_package(Eigen3)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(${CMAKE_SOURCE_DIR}/include)

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

add_executable(main src/test_main.cpp
${CMAKE_SOURCE_DIR}/src/tools.cpp
# ${CMAKE_SOURCE_DIR}/include/tools.h
${CMAKE_SOURCE_DIR}/src/pclProcess.cpp
# ${CMAKE_SOURCE_DIR}/include/pclProcess.h
${CMAKE_SOURCE_DIR}/src/pclSeg.cpp
# ${CMAKE_SOURCE_DIR}/include/pclSeg.h
${CMAKE_SOURCE_DIR}/src/pclCluster.cpp
# ${CMAKE_SOURCE_DIR}/include/pclCluster.h


)

target_link_libraries(main ${PCL_LIBRARIES})
target_link_libraries(main ${OpenCV_LIBS})

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
