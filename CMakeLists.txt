cmake_minimum_required(VERSION 3.10.0)
project(drone_pathgen VERSION 0.1.0 LANGUAGES C CXX)

# Set cmake policies to suppress warnings
cmake_policy(SET CMP0167 NEW)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Export compile commands for IDE support
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Add cmake module path for finding packages
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/opt/homebrew/Cellar/ompl/1.7.0_1/share/ompl/cmake")

# Find required packages
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem serialization system)
find_package(ompl REQUIRED)

# Include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(/opt/homebrew/include/eigen3)
include_directories(${OMPL_INCLUDE_DIRS})

# Add library
add_library(drone_pathgen drone_pathgen.cpp)

# Link libraries
target_link_libraries(drone_pathgen 
    Eigen3::Eigen
    ${Boost_LIBRARIES}
    ${OMPL_LIBRARIES}
)

# Compiler flags
target_compile_options(drone_pathgen PRIVATE ${OMPL_CFLAGS_OTHER})

# For debugging
set(CMAKE_BUILD_TYPE Debug)

add_executable(test test.cpp)
target_link_libraries(test
    drone_pathgen
    Eigen3::Eigen
    ${Boost_LIBRARIES}
    ${OMPL_LIBRARIES}
)

