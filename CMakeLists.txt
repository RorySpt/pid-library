cmake_minimum_required(VERSION 3.31)
project(pid_library)

set(CMAKE_CXX_STANDARD 26)


add_subdirectory(imgui)


add_library(pid pid/PID.h pid/PID.cpp)



add_executable(pid_library main.cpp
        imgui-windows.cpp
)

target_link_libraries(pid_library PUBLIC
        imgui
        pid
)

#find_package(fmt CONFIG REQUIRED)
#target_link_libraries(pid_library PRIVATE fmt::fmt)

# Or use the header-only version
find_package(fmt CONFIG REQUIRED)
target_link_libraries(pid_library PRIVATE fmt::fmt-header-only)

find_package(glfw3 CONFIG REQUIRED)
target_link_libraries(pid_library PRIVATE glfw)

find_package(OpenGL REQUIRED COMPONENTS
        OpenGL) # v3.10
target_link_libraries(pid_library PRIVATE OpenGL::GL)

#target_compile_definitions(pid_library PRIVATE "main=ImguiMain")