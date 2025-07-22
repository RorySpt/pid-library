set(_target "imgui")
macro(printlist list)
    message("search " ${list} )
    foreach(v ${${list}})
        message("found " ${v} )
    endforeach()
endmacro()


#add_library(${_target})

set(imgui-root "imgui-1.92.1")

file(GLOB BACKENDS_CPP_FILES
        ${imgui-root}/backends/imgui_impl_glfw.cpp
        ${imgui-root}/backends/imgui_impl_opengl3*.cpp
)
file(GLOB BACKENDS_H_FILES
        ${imgui-root}/backends/imgui_impl_glfw.h
        ${imgui-root}/backends/imgui_impl_opengl3*.h
)

file(GLOB IMGUI_H_FILES
        ${imgui-root}/*.h
)
#printlist(IMGUI_H_FILES)
#message("search " SANDBOX_IMGUI_CPP_FILES )
#foreach(CPP_FILE ${SANDBOX_IMGUI_CPP_FILES})
#    message("found " ${CPP_FILE} )
#endforeach()

add_library(
        ${_target}
        ${imgui-root}/imconfig.h
        ${imgui-root}/imgui_demo.cpp
        ${imgui-root}/imgui_draw.cpp
        ${imgui-root}/imgui_tables.cpp
        ${imgui-root}/imgui_widgets.cpp
        ${imgui-root}/imgui.cpp
        ${BACKENDS_CPP_FILES}
        ${BACKENDS_H_FILES}
)


target_include_directories(${_target} PUBLIC ${imgui-root} ${imgui-root}/backends)

#find_package(Freetype REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
#find_package(glm CONFIG REQUIRED)

#target_include_directories(${_target} PRIVATE ${VCPKH})
target_link_libraries(${_target} PRIVATE glfw)