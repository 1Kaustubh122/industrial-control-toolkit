cmake_minimum_required(VERSION 3.22)
project(industrial-control-toolkit VERSION 0.1.0 LANGUAGES CXX)

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

add_library(ictk_core INTERFACE)
target_include_directories(ictk_core INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include")
ictk_apply_compiler_options(ictk_core)
ictk_apply_sanitizers(ictk_core)


## TESTING
if (CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
  include(CTest)
  if (BUILD_TESTING)
    add_subdirectory(tests)
  endif() 
endif()

## Install
install(TARGETS ictk_core EXPORT ICTKTargets)
install(EXPORT ICTKTargets NAMESPACE ictk:: DESTINATION lib/cmake/ictk)