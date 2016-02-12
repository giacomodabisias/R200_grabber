cmake_minimum_required(VERSION 2.8)
Project (real_sense CXX)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/modules")
set(CMAKE_CXX_FLAGS   "${CMAKE_CXX_FLAGS} -std=c++11") 
set(CMAKE_CXX_FLAGS   "${CMAKE_CXX_FLAGS} -march=native")
set(CMAKE_BUILD_TYPE   Release) 

set(CMAKE_PREFIX_PATH /prefix)
set(SRC                ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(INCLUDE            ${CMAKE_CURRENT_SOURCE_DIR}/include)

find_package(OpenMP)
if (OPENMP_FOUND)
	set (CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")
	set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${INCLUDE})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(real_sense   ${SRC}/main.cpp )

target_link_libraries(real_sense 	    ${OpenCV_LIBS} 
										${PCL_LIBRARIES}
										DSAPI
										log4cpp
										)



