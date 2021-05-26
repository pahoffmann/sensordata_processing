cmake_minimum_required(VERSION 3.9)
project(3ds)
set(CMAKE_CXX_STANDARD 17)

find_package(Git QUIET REQUIRED)
if(GIT_FOUND AND EXISTS "${PROJECT_SOURCE_DIR}/.git")
    option(GIT_SUBMODULE "Check submodules during build" ON)
    if(GIT_SUBMODULE AND NOT EXISTS "${PROJECT_SOURCE_DIR}/opencv/CMakeLists.txt")
        message(STATUS "Submodule update")
        execute_process(COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive
                        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                        RESULT_VARIABLE GIT_SUBMOD_RESULT)
        if(NOT GIT_SUBMOD_RESULT EQUAL "0")
            message(FATAL_ERROR "git submodule update --init failed with ${GIT_SUBMOD_RESULT}, please checkout submodules")
        endif()
        execute_process(COMMAND ${GIT_EXECUTABLE} checkout 3.4 
                        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/opencv)
    endif()
endif()

if(NOT EXISTS "${PROJECT_SOURCE_DIR}/opencv/CMakeLists.txt")
    message(FATAL_ERROR "The submodules were not downloaded! GIT_SUBMODULE was turned off or failed. Please update submodules and try again.")
endif()

#add_subdirectory(u1)
#add_subdirectory(u2)
add_subdirectory(u3)