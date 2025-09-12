find_package(GTest REQUIRED)
include(GoogleTest)

macro(mesh_sampling_test NAME)
  add_executable(${NAME} ${NAME}.cpp)
  target_link_libraries(${NAME} PUBLIC GTest::gtest ${ARGN})
  target_compile_definitions(
    ${NAME} PRIVATE -DPROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
  gtest_discover_tests(${NAME})
endmacro()

mesh_sampling_test(testWeightedRandomSampling mesh_sampling)
