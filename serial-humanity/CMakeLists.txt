cmake_minimum_required (VERSION 3.12)
set(CMAKE_CXX_STANDARD 11)

project(serial-humanity)
# The version number.
set (Tutorial_VERSION_MAJOR 1)
set (Tutorial_VERSION_MINOR 0)
find_package(Threads REQUIRED)
# configure a header file to pass some of the CMake settings
# to the source code
configure_file (
  "${PROJECT_SOURCE_DIR}/TutorialConfig.h.in"
  "${PROJECT_BINARY_DIR}/TutorialConfig.h"
)

# add the binary tree to the search path for include files
# so that we will find TutorialConfig.h
include_directories("${PROJECT_BINARY_DIR}") 

#add a macro that could be used in the source files
add_definitions(-DMACROS_DEFINE)  #for c++ macro
#set (MACROS_DEFINE True)
if(${MACROS_DEFINE})
  message(STATUS "add a MACROS_DEFINE macro for sources file")
endif (${MACROS_DEFINE})
add_subdirectory(Helper)
# should we use our own math functions?
#option (USE_MYMATH "Use tutorial provided math implementation" ON) 
#if (USE_MYMATH)
#  message(STATUS "using USE_MYMATH for sources file")
#  include_directories ("${PROJECT_SOURCE_DIR}/serial")
#  add_subdirectory (serial)
#  set (EXTRA_LIBS ${EXTRA_LIBS} serial)
#endif (USE_MYMATH)

add_executable(${PROJECT_NAME} main.cpp cserial.cpp)
target_link_libraries (${PROJECT_NAME} ${EXTRA_LIBS} Helper ${CMAKE_THREAD_LIBS_INIT}) 

#install tagets to bin and install file to include
# add the install targets
install (TARGETS ${PROJECT_NAME} DESTINATION bin)
install (FILES "${PROJECT_BINARY_DIR}/TutorialConfig.h"  DESTINATION include)
