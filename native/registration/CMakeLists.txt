cmake_minimum_required(VERSION 3.16)
project(cad_registration VERSION 1.0.0 LANGUAGES CXX)

option(CADREG_BUILD_EXAMPLE "Build the C API example" ON)

# Require the requested PCL version. A newer incompatible installation is not
# silently selected; point PCL_DIR at PCL 1.14.1 when configuring.
find_package(PCL 1.14.1 EXACT REQUIRED COMPONENTS
  common filters kdtree search features sample_consensus registration
)

add_library(cad_registration SHARED src/cad_registration.cpp)
add_library(cadreg::cad_registration ALIAS cad_registration)

target_compile_features(cad_registration PUBLIC cxx_std_14)
target_include_directories(cad_registration
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${PCL_INCLUDE_DIRS}
)
target_link_directories(cad_registration PRIVATE ${PCL_LIBRARY_DIRS})
target_link_libraries(cad_registration PRIVATE ${PCL_LIBRARIES})
target_compile_definitions(cad_registration PRIVATE CADREG_BUILDING_DLL=1)
target_compile_options(cad_registration PRIVATE ${PCL_DEFINITIONS})
set_target_properties(cad_registration PROPERTIES
  OUTPUT_NAME cad_registration
  CXX_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN YES
  WINDOWS_EXPORT_ALL_SYMBOLS OFF
)

if(MSVC)
  target_compile_options(cad_registration PRIVATE /W4 /permissive- /EHsc /utf-8)
else()
  target_compile_options(cad_registration PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(CADREG_BUILD_EXAMPLE)
  add_executable(cadreg_example examples/c_api_example.cpp)
  target_compile_features(cadreg_example PRIVATE cxx_std_14)
  target_link_libraries(cadreg_example PRIVATE cadreg::cad_registration)

  add_executable(cadreg_test examples/registration_test.cpp)
  target_compile_features(cadreg_test PRIVATE cxx_std_17)
  target_link_libraries(cadreg_test PRIVATE cadreg::cad_registration)
  if(MSVC)
    target_compile_options(cadreg_test PRIVATE /W4 /permissive- /EHsc /utf-8)
  endif()
endif()

include(GNUInstallDirs)
install(TARGETS cad_registration
  EXPORT cadreg-targets
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT cadreg-targets
  FILE cadreg-targets.cmake
  NAMESPACE cadreg::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/cadreg
)
