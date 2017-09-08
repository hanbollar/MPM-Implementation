
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
set(EIGEN_INSTALL_DIR ${CMAKE_BINARY_DIR}/include/Eigen3 PARENT_SCOPE)

ExternalProject_Add(tinyobjloader
    GIT_REPOSITORY git@github.com:syoyo/tinyobjloader.git
    GIT_TAG v1.0.6
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}
)

ExternalProject_Add(glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG 3.1
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}
)

include(${CMAKE_BINARY_DIR}/lib/cmake/glfw/glfw3Config.cmake)
