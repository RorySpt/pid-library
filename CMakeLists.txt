cmake_minimum_required(VERSION 3.31)
project(pid_library)

set(CMAKE_CXX_STANDARD 26)


add_subdirectory(imgui)

add_executable(pid_library main.cpp
        pid.cpp
        pid.h
        imgui
)




#find_package(fmt CONFIG REQUIRED)
#target_link_libraries(pid_library PRIVATE fmt::fmt)

# Or use the header-only version
find_package(fmt CONFIG REQUIRED)
target_link_libraries(pid_library PRIVATE fmt::fmt)

find_package(glfw3 CONFIG REQUIRED)
target_link_libraries(pid_library PRIVATE glfw)


target_compile_definitions(pid_library PRIVATE "main=ImguiMain")