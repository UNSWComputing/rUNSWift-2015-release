cmake_minimum_required(VERSION 2.8.0 FATAL_ERROR)

find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CTC_DIR}/libnaoqi/include)
include_directories(${CTC_DIR}/bzip2/include)
include_directories(${CTC_DIR}/zlib/include)

include("${CMAKE_CURRENT_SOURCE_DIR}/libagent.cmake")
include("${CMAKE_CURRENT_SOURCE_DIR}/runswift.cmake")
include("${CMAKE_CURRENT_SOURCE_DIR}/robot.cmake")

include("${CMAKE_CURRENT_SOURCE_DIR}/testrunswift.cmake")
include("${CMAKE_CURRENT_SOURCE_DIR}/benchrunswift.cmake")
cotire(agent robot-static runswift soccer-static soccer)
