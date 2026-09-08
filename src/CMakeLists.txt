add_library(mesh_sampling SHARED mesh_sampling.cpp weighted_random_sampling.cpp)

target_link_libraries(mesh_sampling PUBLIC mesh_sampling::assimp Eigen3::Eigen)

if(WIN32)
  target_link_libraries(mesh_sampling PRIVATE Qhull::qhullcpp Qhull::qhullstatic_r)
else()
  target_link_libraries(mesh_sampling PRIVATE Qhull::qhullcpp Qhull::qhull_r)
endif()


target_include_directories(
  mesh_sampling PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
                       $<INSTALL_INTERFACE:include>)

target_include_directories(mesh_sampling
                           PRIVATE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/ext>)

# Propagate the configuration to your compiler flags
if(USE_LEGACY_QHULL_STREAM)
  message(STATUS "Qhull Stream Strategy: LEGACY (open_memstream)")
  target_compile_definitions(mesh_sampling PRIVATE USE_LEGACY_QHULL_STREAM=1)
else()
  message(STATUS "Qhull Stream Strategy: MODERN (std::stringstream)")
  target_compile_definitions(mesh_sampling PRIVATE USE_LEGACY_QHULL_STREAM=0)
endif()

install(
  TARGETS mesh_sampling
  EXPORT "${TARGETS_EXPORT_NAME}"
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/ DESTINATION include)
