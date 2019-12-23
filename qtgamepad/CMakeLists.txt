cmake_minimum_required(VERSION 3.5)
project(qtgamepad)
SET(CMAKE_INCLUDE_CURRENT_DIR ON)

execute_process(COMMAND ${CMAKE_SOURCE_DIR}/scripts/find_qt_path.sh OUTPUT_VARIABLE CMAKE_PREFIX_PATH_1)
SET(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH_1})

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
find_package(Qt5Core CONFIG REQUIRED)
find_package(Qt5Widgets CONFIG REQUIRED)
find_package(Qt5Gui CONFIG REQUIRED)

get_target_property(QtCore_location Qt5::Core LOCATION)
message(STATUS ${QtCore_location})



file(GLOB sources "src/gamepad/*.cpp")
add_library(qtgamepad SHARED ${sources})
