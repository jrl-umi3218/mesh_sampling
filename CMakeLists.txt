cmake_minimum_required(VERSION 3.11)

set(PROJECT_NAME mesh_sampling)
set(PROJECT_DESCRIPTION "Sample pointcloud from meshes")
set(PROJECT_URL https://github.com/jrl-umi3218/mesh_sampling)
set(PROJECT_VERSION 1.0.0)
set(PROJECT_USE_CMAKE_EXPORT TRUE)
set(PROJECT_DEBUG_POSTFIX "_d")
set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_STANDARD 17)

include(cmake/base.cmake)

project(
  ${PROJECT_NAME}
  LANGUAGES CXX
  VERSION ${PROJECT_VERSION})

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMakeModules)

# Note: Since we are using custom cmake scripts to look at
# mesh_sampling_3rd_party_assimp and create clean modern cmake targets, using
# add_project_dependency leads to the mesh_samplingConfig script to try and find
# these dependencies before the path to the required
# mesh_sampling_3rd_party_*.cmake have been included.
find_package(mesh_sampling_3rd_party_assimp REQUIRED)
find_package(CLI11 REQUIRED)
add_project_dependency(Eigen3 REQUIRED NO_MODULE)
add_project_dependency(Qhull REQUIRED)

install(
  FILES CMakeModules/Findmesh_sampling_3rd_party_assimp.cmake
  DESTINATION lib/cmake/mesh_sampling
  RENAME mesh_sampling_3rd_party_assimpConfig.cmake)

# Export custom package dependencies to the mesh_samplingConfig script such that
# they are found when importing the package with find_package(mesh_sampling)
set(PACKAGE_EXTRA_MACROS
    "include(\"\$\{CMAKE_CURRENT_LIST_DIR\}/mesh_sampling_3rd_party_assimpConfig.cmake\")"
)

add_subdirectory(src)
add_subdirectory(utils)

if(${BUILD_TESTING})
  add_subdirectory(tests)
endif()
