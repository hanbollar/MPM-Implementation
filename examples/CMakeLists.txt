
file(GLOB EXAMPLES *.cc)

find_package(Threads)

foreach(EXAMPLE ${EXAMPLES})
    get_filename_component(NAME ${EXAMPLE} NAME_WE)
    add_executable(${NAME} ${EXAMPLE})
    target_link_libraries(${NAME} flint flint_viewport ${CMAKE_THREAD_LIBS_INIT})
endforeach()
