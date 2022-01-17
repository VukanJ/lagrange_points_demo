cmake_minimum_required(VERSION 3.11)

project(planets)

set(CMAKE_CXX_STANDARD_REQUIRED 17)
add_compile_options(-std=c++2a -Wall -Wpedantic -Wextra -Wunused-value -Wunused-function -Wshadow)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -Ofast")

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(SFML 2.5 COMPONENTS graphics window system REQUIRED)
set(LIBRARIES Eigen3::Eigen sfml-graphics sfml-window sfml-system)

file(GLOB SRC "*.cpp")

if (SFML_FOUND)
    add_executable(planets ${SRC})
    target_link_libraries(planets PRIVATE ${LIBRARIES})
else ()
    message(FATAL_ERROR "Could not resolve dependencies")
endif ()
