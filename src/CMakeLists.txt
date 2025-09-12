add_library(mesh_sampling SHARED mesh_sampling.cpp weighted_random_sampling.cpp)

target_link_libraries(
  mesh_sampling PUBLIC mesh_sampling::assimp Qhull::qhullcpp Qhull::qhull_r
                       Eigen3::Eigen)

if(TARGET Eigen3::Eigen)
  target_link_libraries(mesh_sampling PUBLIC Eigen3::Eigen)
else() # legacy for older eigen installations, this is most likely no longer
       # needed
  target_include_directories(mesh_sampling SYSTEM ${Eigen_INCLUDE_DIR})
endif()
target_include_directories(
  mesh_sampling PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
                       $<INSTALL_INTERFACE:include>)

target_include_directories(mesh_sampling
                           PRIVATE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/ext>)

install(
  TARGETS mesh_sampling
  EXPORT "${TARGETS_EXPORT_NAME}"
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/ DESTINATION include)
