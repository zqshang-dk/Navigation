cmake_minimum_required(VERSION 3.10)

project(SPP LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Keep the original output convention so the generated binary lands in ./lib.
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build." FORCE)
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(Threads REQUIRED)
find_package(OpenMP QUIET)
find_package(Eigen3 QUIET NO_MODULE)

set(EIGEN3_FALLBACK_INCLUDE_DIR "/home/sd1/personal/zqshang/UsefulTools/eigen-3.3.8" CACHE PATH "Fallback Eigen include directory")

if(OpenMP_CXX_FOUND)
    message(STATUS "OpenMP found")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

file(GLOB PROJECT_SOURCES CONFIGURE_DEPENDS
    "${PROJECT_SOURCE_DIR}/*.cpp"
)

add_executable(SPP ${PROJECT_SOURCES})

target_include_directories(SPP PRIVATE
    ${PROJECT_SOURCE_DIR}
)

if(Eigen3_FOUND)
    target_link_libraries(SPP PRIVATE Eigen3::Eigen)
else()
    # Fallbacks for servers where Eigen is unpacked manually instead of installed system-wide.
    target_include_directories(SPP PRIVATE
        /usr/include/eigen3
        ${EIGEN3_FALLBACK_INCLUDE_DIR}
    )
endif()

target_link_libraries(SPP PRIVATE Threads::Threads)

if(WIN32)
    target_link_libraries(SPP PRIVATE ws2_32)
endif()

target_compile_options(SPP PRIVATE
    -g
)
