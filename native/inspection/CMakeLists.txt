cmake_minimum_required(VERSION 3.16)
project(cad_inspection VERSION 1.0.0 LANGUAGES CXX)

option(CADINSPECT_BUILD_EXAMPLE "Build inspection test" ON)

add_library(cad_inspection SHARED src/cad_inspection.cpp)
target_compile_features(cad_inspection PUBLIC cxx_std_17)
target_include_directories(cad_inspection PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(cad_inspection PRIVATE CADINSPECT_BUILDING_DLL=1)
set_target_properties(cad_inspection PROPERTIES
  OUTPUT_NAME cad_inspection
  CXX_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN YES
  WINDOWS_EXPORT_ALL_SYMBOLS OFF)
if(MSVC)
  target_compile_options(cad_inspection PRIVATE /W4 /permissive- /EHsc /utf-8)
else()
  target_compile_options(cad_inspection PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_library(cad_inspection_wide SHARED src/cad_inspection.cpp)
target_compile_features(cad_inspection_wide PUBLIC cxx_std_17)
target_include_directories(cad_inspection_wide PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(cad_inspection_wide PRIVATE
  CADINSPECT_BUILDING_DLL=1 CADINSPECT_DEFAULT_WIDE=1)
set_target_properties(cad_inspection_wide PROPERTIES
  OUTPUT_NAME cad_inspection_wide
  CXX_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN YES
  WINDOWS_EXPORT_ALL_SYMBOLS OFF)
if(MSVC)
  target_compile_options(cad_inspection_wide PRIVATE /W4 /permissive- /EHsc /utf-8)
else()
  target_compile_options(cad_inspection_wide PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(CADINSPECT_BUILD_EXAMPLE)
  add_executable(cadinspect_test examples/inspection_test.cpp)
  target_compile_features(cadinspect_test PRIVATE cxx_std_17)
  target_link_libraries(cadinspect_test PRIVATE cad_inspection)

  add_executable(cadinspect_wide_test examples/inspection_test.cpp)
  target_compile_features(cadinspect_wide_test PRIVATE cxx_std_17)
  target_link_libraries(cadinspect_wide_test PRIVATE cad_inspection_wide)

  add_executable(cadinspect_ply_test examples/ply_inspection_test.cpp)
  target_compile_features(cadinspect_ply_test PRIVATE cxx_std_17)
  target_link_libraries(cadinspect_ply_test PRIVATE cad_inspection)
  if(MSVC)
    target_compile_options(cadinspect_ply_test PRIVATE /W4 /permissive- /EHsc /utf-8)
  endif()
endif()
