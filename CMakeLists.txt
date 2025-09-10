cmake_minimum_required(VERSION 3.22)
project(industrial-control-toolkit VERSION 0.0.2 LANGUAGES CXX)

# Build Options
## Exception handling (Throw)
option(ICTK_NO_EXCEPTIONS "Build without exceptions" ON)
## Runtime Type Information RTTI (Enable/disable dynamic_cast and typeid)
option(ICTK_NO_RTTI "Build without RTTI" ON)
## ASAN (Enable/Disable Asan) -> Checks for memory errors if activated
option(ICTK_ENABLE_ASAN "Enable AddressSanitizer" OFF)
## UBSAN (Enable/Disable Ubsan) -> Checks for undefined behavior like overflow, division by zero, out of bound etc if activated
option(ICTK_ENABLE_UBSAN "Enable UndefinedBehaviorSanitizer" OFF)
## Link Time Optimization (LTO) -> Optimizes the whole program by inline functions, dead code elimination, etc if activated
option(ICTK_ENABLE_LTO "Enable Link Time Optimisation" OFF)
## Build with benchmark
option(ICTK_BUILD_BENCHMARKS "Build ICTK benchmark runner" ON)

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Debug or Release" FORCE)
endif()

# C++ Standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules")
include(ICTKCompilerOptions)
include(ICTKSanitizers)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
include(CheckIPOSupported)

add_library(ictk STATIC
  src/ictk_version.cc
)

add_library(ictk_core ALIAS ictk)
add_library(ictk::ictk ALIAS ictk)

target_include_directories(ictk 
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(ictk PUBLIC cxx_std_20)

ictk_apply_compiler_options(ictk)
ictk_apply_sanitizers(ictk)

if(ICTK_ENABLE_LTO)
  check_ipo_supported(RESULT ipo_ok OUTPUT ipo_msg)
  if(ipo_ok)
    set_property(TARGET ictk PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
  else()
    message(WARNING "IPO/LTO not supported: ${ipo_msg}")
  endif()
endif()

## Benches
if (ICTK_BUILD_BENCHMARKS)
  add_subdirectory(benchmarks)
endif()

## TESTING
if (CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
  include(CTest)
  if (BUILD_TESTING)
    add_subdirectory(tests)
  endif() 
endif()

## Install
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(TARGETS ictk
  EXPORT ictkTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(EXPORT ictkTargets
  NAMESPACE ictk::
  FILE ictkTargets.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ICTK
)

## Package Config
configure_package_config_file(
  cmake/ICTKConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/ICTKConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ICTK
  NO_CHECK_REQUIRED_COMPONENTS_MACRO
)

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/ICTKConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/ICTKConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/ICTKConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ICTK
)

## Cpack
set(CPACK_PACKAGE_NAME "ictk-dev")
set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
set(CPACK_GENERATOR "DEB")
set(CPACK_DEBIAN_FILE_NAME DEB-DEFAULT)
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Kaustubh <kkaustubh2006@gmail.com>")
set(CPACK_DEBIAN_PACKAGE_SECTION "libs")
set(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")
set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)
include(CPack)