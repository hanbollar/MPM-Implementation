
include(ExternalProject)

if (WIN32)
    # By default, ExternalProject finds the wrong git
    find_package(Git REQUIRED)
    set(GIT_EXECUTABLE "C:/Program Files/Git/cmd/git.exe")
endif()

ExternalProject_Add(eigen
    GIT_REPOSITORY https://github.com/RLovelett/eigen.git
    GIT_TAG 3.3.4
    UPDATE_COMMAND ""
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND
        ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_BINARY_DIR}/third_party/eigen-prefix/src/eigen/Eigen
            ${CMAKE_BINARY_DIR}/include/Eigen3/Eigen &&
        ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_BINARY_DIR}/third_party/eigen-prefix/src/eigen/unsupported
            ${CMAKE_BINARY_DIR}/include/Eigen3/unsupported
)
set(EIGEN_INCLUDE_DIR ${CMAKE_BINARY_DIR}/include/Eigen3 PARENT_SCOPE)

ExternalProject_Add(tinyobjloader
    GIT_REPOSITORY https://github.com/syoyo/tinyobjloader.git
    GIT_TAG v1.0.6
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}
)

ExternalProject_Add(GLAD
    GIT_REPOSITORY https://github.com/Dav1dde/glad.git
	GIT_TAG v0.1.14a0
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR} -DGLAD_INSTALL:BOOL=ON
)

add_library(glad STATIC IMPORTED GLOBAL)
set_target_properties(glad PROPERTIES IMPORTED_LOCATION ${CMAKE_BINARY_DIR}/lib/${CMAKE_STATIC_LIBRARY_PREFIX}glad${CMAKE_STATIC_LIBRARY_SUFFIX})

ExternalProject_Add(GLFW
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG 3.2.1
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}
)

add_library(glfw STATIC IMPORTED GLOBAL)
set_target_properties(glfw PROPERTIES IMPORTED_LOCATION ${CMAKE_BINARY_DIR}/lib/${CMAKE_STATIC_LIBRARY_PREFIX}glfw3${CMAKE_STATIC_LIBRARY_SUFFIX})
set(GLFW_INCLUDE_DIR ${CMAKE_BINARY_DIR}/include PARENT_SCOPE)

ExternalProject_Add(png
    GIT_REPOSITORY https://github.com/glennrp/libpng.git
    GIT_TAG libpng16
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}
)

add_library(png16 STATIC IMPORTED GLOBAL)
set_target_properties(png16 PROPERTIES IMPORTED_LOCATION ${CMAKE_BINARY_DIR}/lib/${CMAKE_STATIC_LIBRARY_PREFIX}png16${CMAKE_STATIC_LIBRARY_SUFFIX})
set(PNG_INCLUDE_DIR ${CMAKE_BINARY_DIR}/include PARENT_SCOPE)

ExternalProject_Add(googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG release-1.8.0
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}
)

add_library(gtest STATIC IMPORTED GLOBAL)
set_target_properties(gtest PROPERTIES IMPORTED_LOCATION ${CMAKE_BINARY_DIR}/lib/${CMAKE_STATIC_LIBRARY_PREFIX}gtest${CMAKE_STATIC_LIBRARY_SUFFIX})
