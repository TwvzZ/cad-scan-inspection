cmake_minimum_required(VERSION 3.16)
project(cad_sampling VERSION 1.0.0 LANGUAGES CXX)

option(CADSAMPLE_BUILD_EXAMPLE "Build the C API example" ON)

find_package(OpenCASCADE 7.9.3 EXACT REQUIRED)

add_library(cad_sampling SHARED src/cad_sampling.cpp)
add_library(cadsample::cad_sampling ALIAS cad_sampling)
target_compile_features(cad_sampling PUBLIC cxx_std_17)
target_include_directories(cad_sampling
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${OpenCASCADE_INCLUDE_DIR}
)
target_link_directories(cad_sampling PRIVATE ${OpenCASCADE_LIBRARY_DIR})
target_link_libraries(cad_sampling PRIVATE
  TKernel TKMath TKG2d TKG3d TKGeomBase TKBRep TKGeomAlgo
  TKTopAlgo TKMesh TKXSBase TKDESTEP
)
target_compile_definitions(cad_sampling PRIVATE CADSAMPLE_BUILDING_DLL=1)
set_target_properties(cad_sampling PROPERTIES
  OUTPUT_NAME cad_sampling
  CXX_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN YES
  WINDOWS_EXPORT_ALL_SYMBOLS OFF
)

if(MSVC)
  target_compile_options(cad_sampling PRIVATE /W4 /permissive- /EHsc /utf-8)
else()
  target_compile_options(cad_sampling PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(CADSAMPLE_BUILD_EXAMPLE)
  add_executable(cadsample_example examples/c_api_example.cpp)
  target_compile_features(cadsample_example PRIVATE cxx_std_17)
  target_link_libraries(cadsample_example PRIVATE cadsample::cad_sampling)

  add_executable(cadsample_test examples/sampling_test.cpp)
  target_compile_features(cadsample_test PRIVATE cxx_std_17)
  target_link_libraries(cadsample_test PRIVATE cadsample::cad_sampling)
endif()

include(GNUInstallDirs)
install(TARGETS cad_sampling
  EXPORT cadsample-targets
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT cadsample-targets
  FILE cadsample-targets.cmake
  NAMESPACE cadsample::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/cadsample
)
