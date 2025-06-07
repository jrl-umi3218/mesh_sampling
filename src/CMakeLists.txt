add_library(mesh_sampling SHARED
  mesh_sampling.cpp
  weighted_random_sampling.cpp
)

target_link_libraries(mesh_sampling PUBLIC mesh_sampling::assimp mesh_sampling::PCL Qhull::qhullcpp Qhull::qhull_r)

target_include_directories(mesh_sampling PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include> 
  $<INSTALL_INTERFACE:include>
)

target_include_directories(mesh_sampling PRIVATE 
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/ext>
)

install(TARGETS mesh_sampling 
  EXPORT "${TARGETS_EXPORT_NAME}"
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(
  DIRECTORY ${CMAKE_SOURCE_DIR}/include/ DESTINATION include)