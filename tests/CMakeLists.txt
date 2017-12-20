
set(FLINT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/flint)

if (WIN32)
    add_compile_options(-MTd)
endif()

add_executable(flint_tests
    ${FLINT_DIR}/intersection/AxisAlignedBox.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/main.cc
)
add_dependencies(flint_tests googletest)
target_link_libraries(flint_tests flint gtest)

