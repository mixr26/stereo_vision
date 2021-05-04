cmake_minimum_required(VERSION 3.10)

set(SOURCE_PATH ${PROJECT_SOURCE_DIR}/source)

find_package(OpenCV REQUIRED)

set(OpenCV_LIBS "-lopencv_core -lopencv_highgui -lopencv_imgcodecs -lopencv_imgproc -lopencv_calib3d")

file(GLOB_RECURSE SRC_FILES ${SOURCE_PATH}/*.cpp)
file(GLOB_RECURSE INC_FILES ${SOURCE_PATH}/*.hpp)

include_directories(${SOURCE_PATH}/inc
                    ${OpenCV_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2")

set(CMAKE_VERBOSE_MAKEFILE true)

add_executable(${APP_NAME} ${SRC_FILES} ${INC_FILES})
target_link_libraries(${APP_NAME} ${OpenCV_LIBS})
set_target_properties(${APP_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}")
