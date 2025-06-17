include(CMakePackageConfigHelpers)

# Set install paths
set(CONFIG_INSTALL_DIR lib/cmake/mesh_sampling)

# Generate the config file
configure_package_config_file(
  mesh_samplingConfig.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/mesh_samplingConfig.cmake"
  INSTALL_DESTINATION ${CONFIG_INSTALL_DIR})

# Optionally generate version file
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/mesh_samplingConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion)

# Install the generated config files
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/mesh_samplingConfig.cmake"
              "${CMAKE_CURRENT_BINARY_DIR}/mesh_samplingConfigVersion.cmake"
        DESTINATION ${CONFIG_INSTALL_DIR})
