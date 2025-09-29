cmake_minimum_required(VERSION 3.22)
project(ictk_evidence_recorder LANGUAGES CXX)

# # Default ON; MCAP default OFF
option(ICTK_BUILD_RECORDER "Build ICTK Evidence Recorder tools" ON)
option(ICTK_RECORDER_BACKEND_MCAP "Enable Mcap+FlatBuffers backend (requires deps)" OFF)

# # Exit if no record
if(NOT ICTK_BUILD_RECORDER)
    return()
endif()

# # static lib target
add_library(ictk_recorder STATIC
    ${CMAKE_CURRENT_LIST_DIR}/src/recorder_jsonl.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/kpi_calc.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/env_buildinfo.cpp
)

# # this traget compile to c++ 20
target_compile_features(ictk_recorder PRIVATE cxx_std_20)

# # Emits PIC (-fPIC) for this lib (for linux)
set_target_properties(ictk_recorder PROPERTIES POSITION_INDEPENDENT_CODE ON)

if(ICTK_RECORDER_BACKEND_MCAP)
    include(FetchContent)

    # Fetch MCAP headers
    FetchContent_Declare(
        mcap
        GIT_REPOSITORY https://github.com/foxglove/mcap.git
        GIT_TAG        main
        SOURCE_SUBDIR  cpp/mcap
    )
    FetchContent_MakeAvailable(mcap)

    find_program(FLATC flatc REQUIRED)
    set(SCHEMA  ${CMAKE_CURRENT_LIST_DIR}/schemas/ictk_metrics.fbs)
    set(GEN_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated)
    add_custom_command(
        OUTPUT ${GEN_DIR}/ictk_metrics_generated.h ${GEN_DIR}/ictk_metrics.bfbs
        COMMAND ${FLATC} --cpp --bfbs --bfbs-comments --scoped-enums -o ${GEN_DIR} ${SCHEMA}
        DEPENDS ${SCHEMA}
        VERBATIM
    )

    # Add MCAP sources
    target_sources(ictk_recorder PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/src/recorder_mcap.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/hash.cpp
        ${GEN_DIR}/ictk_metrics_generated.h
    )

    target_link_libraries(ictk_recorder PRIVATE mcap)

    target_compile_definitions(ictk_recorder PUBLIC
        ICTK_RECORDER_BACKEND_MCAP=1
        ICTK_FB_SCHEMA_DIR=\"${GEN_DIR}\"
    )

else()
    target_compile_definitions(ictk_recorder PUBLIC
        ICTK_RECORDER_BACKEND_MCAP=0
        ICTK_FB_SCHEMA_DIR=\"${CMAKE_CURRENT_LIST_DIR}/schemas\"
    )
endif()


target_include_directories(ictk_recorder
    PUBLIC
        # # source build: path included -> PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
        # # install: include headers
        $<INSTALL_INTERFACE:include>
    PRIVATE
        # # Prevent ABI issue
        ${CMAKE_CURRENT_LIST_DIR}/src
)

# # defined macros as 1 or 0
target_compile_definitions(ictk_recorder PUBLIC
    ICTK_RECORDER_BACKEND_MCAP=$<IF:$<BOOL:${ICTK_RECORDER_BACKEND_MCAP}>,1,0>
)

# # Store a configurable schema path in the cache 
set(ICTK_FB_SCHEMA_DIR ${CMAKE_CURRENT_LIST_DIR}/schemas CACHE PATH "FlatBuffers schema dir")
target_compile_definitions(ictk_recorder PUBLIC
    ICTK_FB_SCHEMA_DIR=\"${ICTK_FB_SCHEMA_DIR}\"
)

add_executable(ictk_record
    ${CMAKE_CURRENT_LIST_DIR}/cli/ictk_record_main.cpp
)

# # Not re-expose lib's usage req 
target_link_libraries(ictk_recorder PRIVATE ictk) 
target_link_libraries(ictk_record   PRIVATE ictk_recorder ictk)

# # for CTest
enable_testing()

add_executable(recorder_roundtrip_test
    ${CMAKE_CURRENT_LIST_DIR}/tests/roundtrip_test.cpp
)

target_link_libraries(recorder_roundtrip_test PRIVATE ictk_recorder ictk)
add_test(NAME recorder_roundtrip_test COMMAND recorder_roundtrip_test)

add_executable(recorder_timebase_test
${CMAKE_CURRENT_LIST_DIR}/tests/timebase_test.cpp
)

target_link_libraries(recorder_timebase_test  PRIVATE ictk_recorder ictk)
add_test(NAME recorder_timebase_test COMMAND recorder_timebase_test)


# # Compiler options
if(COMMAND ictk_apply_compiler_options)
    ictk_apply_compiler_options(ictk_recorder)
    ictk_apply_compiler_options(ictk_record)
    ictk_apply_compiler_options(recorder_roundtrip_test)
    ictk_apply_compiler_options(recorder_timebase_test)
endif()