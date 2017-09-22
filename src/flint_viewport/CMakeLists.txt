file(GLOB_RECURSE SOURCES *.cc)
file(GLOB_RECURSE HEADERS *.h)

find_package(OpenGL REQUIRED)
if(NOT OpenGL_FOUND)
    message(ERROR "OpenGL not found")
endif()

list(APPEND FLINT_VIEWPORT_LINK_LIBRARIES
    glfw
    glad
    ${OPENGL_LIBRARIES}
)

if(UNIX AND NOT APPLE)
    find_package(X11 REQUIRED)
    if(NOT X11_FOUND)
        message(ERROR "x11 not found")
    endif()

    list(APPEND FLINT_VIEWPORT_LINK_LIBRARIES
        ${X11_LIBRARIES}
        ${X11_Xxf86vm_LIB}
        ${X11_Xcursor_LIB}
        ${X11_Xrandr_LIB}
        ${X11_Xi_LIB}
        ${X11_Xinerama_LIB}
    )
endif()


add_library(flint_viewport ${SOURCES} ${HEADERS})
set_target_properties(flint_viewport PROPERTIES LINKER_LANGUAGE CXX)
add_dependencies(flint_viewport GLFW GLAD)
target_link_libraries(flint_viewport ${FLINT_VIEWPORT_LINK_LIBRARIES})

set(INCLUDE_DEST "include/flint_viewport")
set(LIB_DEST "lib/${CMAKE_BUILD_TYPE}")

target_include_directories(flint_viewport PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>
  $<BUILD_INTERFACE:${EIGEN_INSTALL_DIR}>
  $<BUILD_INTERFACE:${OPENGL_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${INCLUDE_DEST}>
)

install(TARGETS flint_viewport EXPORT flint_viewport DESTINATION ${LIB_DEST})
install(FILES ${HEADERS} DESTINATION ${INCLUDE_DEST})
