add_executable(mesh_sampling_bin mesh_sampling.cpp)
target_link_libraries(mesh_sampling_bin PUBLIC mesh_sampling)
set_target_properties(mesh_sampling_bin PROPERTIES OUTPUT_NAME mesh_sampling)

install(
  TARGETS mesh_sampling_bin
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)
