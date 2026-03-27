cmake_minimum_required(VERSION 3.14)
project(xsens_mvn_sdk)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread chrono)

file(GLOB SDK_SOURCES src/*.cpp)

add_library(${PROJECT_NAME} SHARED ${SDK_SOURCES})

target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  Boost::system
  Boost::thread
  Boost::chrono
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)