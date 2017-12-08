
file(GLOB EXAMPLES *.cc)

list(APPEND EXAMPLE_LINK_LIBRARIES
    # png16
    flint
    flint_viewport
	simulation
)

find_package(Threads REQUIRED)
list(APPEND EXAMPLE_LINK_LIBRARIES ${CMAKE_THREAD_LIBS_INIT})

if(UNIX AND NOT APPLE)
    find_package(ZLIB REQUIRED)
    list(APPEND EXAMPLE_LINK_LIBRARIES ${ZLIB_LIBRARIES})
endif()

foreach(EXAMPLE ${EXAMPLES})
    get_filename_component(NAME ${EXAMPLE} NAME_WE)
    add_executable(${NAME} ${EXAMPLE})
	# add_dependencies(${NAME} png)
    target_link_libraries(${NAME} ${EXAMPLE_LINK_LIBRARIES})
endforeach()
