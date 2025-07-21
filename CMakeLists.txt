cmake_minimum_required(VERSION 3.31)
project(pid_library)

set(CMAKE_CXX_STANDARD 26)

add_executable(pid_library main.cpp
        pid.cpp
        pid.h)

#find_package(fmt CONFIG REQUIRED)
#target_link_libraries(pid_library PRIVATE fmt::fmt)

# Or use the header-only version
find_package(fmt CONFIG REQUIRED)
target_link_libraries(pid_library PRIVATE fmt::fmt-header-only)