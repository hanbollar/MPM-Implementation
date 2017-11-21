file(GLOB_RECURSE SOURCES *.cc)
file(GLOB_RECURSE HEADERS *.h)

add_library(simulation ${SOURCES} ${HEADERS})
set_target_properties(simulation PROPERTIES LINKER_LANGUAGE CXX)
add_dependencies(simulation eigen)

set(INCLUDE_DEST "include/simulation")
set(LIB_DEST "lib/${CMAKE_BUILD_TYPE}")

target_include_directories(simulation PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>
  $<BUILD_INTERFACE:${EIGEN_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${INCLUDE_DEST}>
)

install(TARGETS simulation EXPORT simulation DESTINATION ${LIB_DEST})
install(FILES ${HEADERS} DESTINATION ${INCLUDE_DEST})
