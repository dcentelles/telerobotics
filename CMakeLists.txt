cmake_minimum_required (VERSION 2.6)
project (telerobotics CXX)


set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_COLOR_MAKEFILE   ON)

# Remove 'lib' prefix for shared libraries on Windows
if (WIN32)
  set(CMAKE_SHARED_LIBRARY_PREFIX "")
endif ()
 
####### LOAD DEPENDENCIES ######

SET(${PROJECT_NAME}_DEPS_DIR "deps/")

if (NOT TARGET dccomms)
	add_subdirectory(${${PROJECT_NAME}_DEPS_DIR}/dccomms)
endif()

if (NOT TARGET cpplogging)
	add_subdirectory(${${PROJECT_NAME}_DEPS_DIR}/cpplogging)
endif()

###### 

SET(${PROJECT_NAME}_INCLUDE_DIRS "includes")
file(GLOB ${PROJECT_NAME}_CPP_FILES "src/*.cpp") 
SET(${PROJECT_NAME}_CXX_COMPILE_FLAGS "-fPIC -std=c++0x")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${${PROJECT_NAME}_CXX_COMPILE_FLAGS}")

add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_CPP_FILES})
target_include_directories(${PROJECT_NAME} PUBLIC ${${PROJECT_NAME}_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} dccomms cpplogging)

file(GLOB ${PROJECT_NAME}_INCLUDES "includes/*.h")
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION "lib")
install(FILES ${${PROJECT_NAME}_INCLUDES} DESTINATION "include")
	
