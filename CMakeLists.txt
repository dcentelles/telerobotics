cmake_minimum_required (VERSION 2.6)
project (telerobotics CXX)


set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_COLOR_MAKEFILE   ON)

# Remove 'lib' prefix for shared libraries on Windows
if (WIN32)
  set(CMAKE_SHARED_LIBRARY_PREFIX "")
endif ()
 
####### LOAD DEPENDENCIES ######

SET(TLROBOTICS_DEPS_DIR "deps/")

if (NOT TARGET dccomms)
	add_subdirectory(${TLROBOTICS_DEPS_DIR}/dccomms)
endif()

if (NOT TARGET cpplogging)
	add_subdirectory(${TLROBOTICS_DEPS_DIR}/cpplogging)
endif()

###### 

SET(TLROBOTICS_INCLUDE_DIRS "includes")
file(GLOB TLROBOTICS_CPP_FILES "src/*.cpp") 
SET(TLROBOTICS_CXX_COMPILE_FLAGS "-fPIC -std=c++0x")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TLROBOTICS_CXX_COMPILE_FLAGS}")
SET(CMAKE_BUILD_TYPE Release)

add_library(${PROJECT_NAME} SHARED ${TLROBOTICS_CPP_FILES})
target_include_directories(${PROJECT_NAME} PUBLIC ${TLROBOTICS_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} dccomms cpplogging)
