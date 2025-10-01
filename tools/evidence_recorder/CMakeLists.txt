cmake_minimum_required(VERSION 3.20)
project(ictk_evidence_recorder LANGUAGES C CXX)

option(ICTK_BUILD_RECORDER "Build ICTK evidence recorder tool" ON)
option(ICTK_RECORDER_BACKEND_MCAP "Enable MCAP+FlatBuffers backend" OFF)
option(ICTK_FETCH_DEPS "Fetch MCAP/FlatBuffers/BLAKE3 locally" ON)

if(NOT ICTK_BUILD_RECORDER)
  return()
endif()

# # Compiler options from root
include(${CMAKE_SOURCE_DIR}/cmake/modules/ICTKCompilerOptions.cmake)

# # Schema paths
set(ICTK_FB_SCHEMA_DIR ${CMAKE_CURRENT_LIST_DIR}/schemas CACHE PATH "FlatBuffers schema dir")
set(ICTK_FB_BFBS ${ICTK_FB_SCHEMA_DIR}/ictk_metrics.bfbs)

# # GIT_SHA 
if(NOT DEFINED GIT_SHA)
  execute_process(
    COMMAND git rev-parse --short=12 HEAD
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_SHA
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
  )

  if(NOT GIT_SHA)
    set(GIT_SHA "unknown")
  endif()

endif()

add_compile_definitions(GIT_SHA="${GIT_SHA}")

# # only when MCAP backend is ON)
set(FLATC_EXE "")
set(MCAP_INCLUDE_DIR "")
set(FLATBUFFERS_INCLUDE_DIRS "")

if(ICTK_RECORDER_BACKEND_MCAP)
  if(ICTK_FETCH_DEPS)
    include(FetchContent)

    # # FlatBuffers 
    FetchContent_Declare(flatbuffers
      GIT_REPOSITORY https://github.com/google/flatbuffers.git
      GIT_TAG        v25.9.23
    )

    FetchContent_MakeAvailable(flatbuffers)
    set(FLATC_EXE $<TARGET_FILE:flatc>)

    set(FLATBUFFERS_INCLUDE_DIRS ${flatbuffers_SOURCE_DIR}/include)

    # # MCAP header-only
    set(ICTK_MCAP_GIT_TAG "releases/cpp/v1.4.2" CACHE STRING "MCAP C++ tag or branch")

    FetchContent_Declare(mcap
      GIT_REPOSITORY https://github.com/foxglove/mcap.git
      GIT_TAG        ${ICTK_MCAP_GIT_TAG}
      GIT_SHALLOW    TRUE
    )

    FetchContent_Populate(mcap)
    set(MCAP_INCLUDE_DIR ${mcap_SOURCE_DIR}/cpp/mcap/include)

    # # BLAKE3 C impl
    FetchContent_Declare(blake3
      GIT_REPOSITORY https://github.com/BLAKE3-team/BLAKE3.git
      GIT_TAG        1.8.2
      GIT_SHALLOW    TRUE
    )

    FetchContent_MakeAvailable(blake3)
    add_library(blake3 STATIC
      ${blake3_SOURCE_DIR}/c/blake3.c
      ${blake3_SOURCE_DIR}/c/blake3_dispatch.c
      ${blake3_SOURCE_DIR}/c/blake3_portable.c
    )

    target_include_directories(blake3 PUBLIC ${blake3_SOURCE_DIR}/c)
    target_compile_definitions(blake3 PRIVATE
      BLAKE3_NO_SSE2
      BLAKE3_NO_SSE41
      BLAKE3_NO_AVX2
      BLAKE3_NO_AVX512
    )

    target_compile_features(blake3 PUBLIC c_std_99)

  else()
    # # System-provided tools/libs
    find_program(FLATC_EXE flatc REQUIRED)
    find_path(MCAP_INCLUDE_DIR NAMES mcap/writer.hpp REQUIRED)

    find_path(FLATBUFFERS_INCLUDE_DIRS NAMES flatbuffers/flatbuffers.h REQUIRED)

    find_library(BLAKE3_LIB NAMES blake3)

    if(NOT BLAKE3_LIB)
      message(FATAL_ERROR "System BLAKE3 not found and ICTK_FETCH_DEPS=OFF. Turn ICTK_FETCH_DEPS ON or provide blake3.")
    endif()

    add_library(blake3 UNKNOWN IMPORTED)
    set_target_properties(blake3 PROPERTIES IMPORTED_LOCATION "${BLAKE3_LIB}")

  endif()

  # # FlatBuffers: build BFBS and generated headers
  add_custom_command(
    OUTPUT ${ICTK_FB_BFBS}
    COMMAND ${FLATC_EXE} --binary --schema -o ${ICTK_FB_SCHEMA_DIR}
            ${ICTK_FB_SCHEMA_DIR}/ictk_metrics.fbs
    DEPENDS ${ICTK_FB_SCHEMA_DIR}/ictk_metrics.fbs
    COMMENT "Generating FlatBuffers BFBS"
    VERBATIM
  )

  add_custom_target(ictk_bfbs DEPENDS ${ICTK_FB_BFBS})

  set(GEN_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated)
  add_custom_command(
    OUTPUT ${GEN_DIR}/ictk_metrics_generated.h
    COMMAND ${FLATC_EXE} --cpp --scoped-enums -o ${GEN_DIR}
            ${ICTK_FB_SCHEMA_DIR}/ictk_metrics.fbs
    DEPENDS ${ICTK_FB_SCHEMA_DIR}/ictk_metrics.fbs
    COMMENT "Generating FlatBuffers C++ headers"
    VERBATIM
  )

  add_custom_target(ictk_fb_cpp DEPENDS ${GEN_DIR}/ictk_metrics_generated.h)

  # # Build minimal MCAP static lib from one TU
  add_library(mcap STATIC ${CMAKE_CURRENT_LIST_DIR}/src/thirdparty/mcap_impl.cpp)
  target_include_directories(mcap SYSTEM PUBLIC ${MCAP_INCLUDE_DIR})
  target_compile_features(mcap PUBLIC cxx_std_17)
  target_compile_definitions(mcap PUBLIC MCAP_COMPRESSION_NO_LZ4 MCAP_COMPRESSION_NO_ZSTD)

endif()

# # Library
set(RECORDER_SRC
  ${CMAKE_CURRENT_LIST_DIR}/src/recorder_jsonl.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/kpi_calc.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/env_buildinfo.cpp
)

if(ICTK_RECORDER_BACKEND_MCAP)
  list(APPEND RECORDER_SRC
    ${CMAKE_CURRENT_LIST_DIR}/src/recorder_mcap.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/hash.cpp
  )
endif()

add_library(ictk_recorder STATIC ${RECORDER_SRC})
target_compile_features(ictk_recorder PUBLIC cxx_std_20)
ictk_apply_compiler_options(ictk_recorder)

# # Public headers for consumers + private src
target_include_directories(ictk_recorder
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/src
)

# # Link to core to get <ictk/...> headers
target_link_libraries(ictk_recorder PUBLIC ictk)

# # Macros
target_compile_definitions(ictk_recorder PUBLIC ICTK_FB_SCHEMA_DIR=\"${ICTK_FB_SCHEMA_DIR}\")
target_compile_definitions(ictk_recorder PUBLIC
  ICTK_RECORDER_BACKEND_MCAP=$<IF:$<BOOL:${ICTK_RECORDER_BACKEND_MCAP}>,1,0>
)

if(ICTK_RECORDER_BACKEND_MCAP)
  add_dependencies(ictk_recorder ictk_bfbs ictk_fb_cpp)

  target_include_directories(ictk_recorder BEFORE PRIVATE
    ${GEN_DIR}
    ${MCAP_INCLUDE_DIR}
    ${FLATBUFFERS_INCLUDE_DIRS}
  )

  if(ICTK_FETCH_DEPS)
    target_link_libraries(ictk_recorder PRIVATE flatbuffers)
  endif()

  target_link_libraries(ictk_recorder PRIVATE mcap blake3)
endif()

# # CLI
add_executable(ictk_record ${CMAKE_CURRENT_LIST_DIR}/cli/ictk_record_main.cpp)
target_link_libraries(ictk_record PRIVATE ictk_recorder)
ictk_apply_compiler_options(ictk_record)

# # TESTS
enable_testing()

add_executable(recorder_roundtrip_test ${CMAKE_CURRENT_LIST_DIR}/tests/roundtrip_test.cpp)
target_link_libraries(recorder_roundtrip_test PRIVATE ictk_recorder)
target_include_directories(recorder_roundtrip_test PRIVATE ${CMAKE_CURRENT_LIST_DIR}/src)
ictk_apply_compiler_options(recorder_roundtrip_test)
add_test(NAME recorder_roundtrip_test COMMAND recorder_roundtrip_test)

add_executable(recorder_timebase_test ${CMAKE_CURRENT_LIST_DIR}/tests/timebase_test.cpp)
target_link_libraries(recorder_timebase_test PRIVATE ictk_recorder)
ictk_apply_compiler_options(recorder_timebase_test)
add_test(NAME recorder_timebase_test COMMAND recorder_timebase_test)

if(ICTK_RECORDER_BACKEND_MCAP)
  add_executable(recorder_schema_registry_test ${CMAKE_CURRENT_LIST_DIR}/tests/schema_registry_test.cpp)
  target_link_libraries(recorder_schema_registry_test PRIVATE ictk_recorder)
  target_include_directories(recorder_schema_registry_test PRIVATE ${CMAKE_CURRENT_LIST_DIR}/src)
  ictk_apply_compiler_options(recorder_schema_registry_test)
  add_test(NAME recorder_schema_registry_test COMMAND recorder_schema_registry_test)

  add_executable(recorder_roundtrip_hash_test ${CMAKE_CURRENT_LIST_DIR}/tests/roundtrip_hash_test.cpp)
  target_link_libraries(recorder_roundtrip_hash_test PRIVATE ictk_recorder)
  target_include_directories(recorder_roundtrip_hash_test PRIVATE ${CMAKE_CURRENT_LIST_DIR}/src)
  ictk_apply_compiler_options(recorder_roundtrip_hash_test)
  add_test(NAME recorder_roundtrip_hash_test COMMAND recorder_roundtrip_hash_test)

  add_executable(recorder_throughput_test ${CMAKE_CURRENT_LIST_DIR}/tests/throughput_test.cpp)
  target_link_libraries(recorder_throughput_test PRIVATE ictk_recorder)
  ictk_apply_compiler_options(recorder_throughput_test)
  add_test(NAME recorder_throughput_test COMMAND recorder_throughput_test)
endif()