
include(ExternalProject)

ExternalProject_Add(eigen
    HG_REPOSITORY https://bitbucket.org/eigen/eigen
    HG_TAG 3.3.4
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

ExternalProject_Add(glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG 3.1
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}
)

add_library(glfw3 STATIC IMPORTED GLOBAL)
set_target_properties(glfw3 PROPERTIES IMPORTED_LOCATION ${CMAKE_BINARY_DIR}/lib/${CMAKE_STATIC_LIBRARY_PREFIX}glfw3${CMAKE_STATIC_LIBRARY_SUFFIX})
set(GLFW_INCLUDE_DIR ${CMAKE_BINARY_DIR}/include PARENT_SCOPE)

ExternalProject_Add(png
    GIT_REPOSITORY https://github.com/glennrp/libpng.git
    GIT_TAG libpng16
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}
)

add_library(png16 STATIC IMPORTED GLOBAL)
set_target_properties(png16 PROPERTIES IMPORTED_LOCATION ${CMAKE_BINARY_DIR}/lib/${CMAKE_STATIC_LIBRARY_PREFIX}png16${CMAKE_STATIC_LIBRARY_SUFFIX})
set(PNG_INCLUDE_DIR ${CMAKE_BINARY_DIR}/include PARENT_SCOPE)
