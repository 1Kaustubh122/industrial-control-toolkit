cmake_minimum_required(VERSION 3.20)
set(ACR_VERSION "0.1.0")
project(ictk-acr VERSION ${ACR_VERSION} LANGUAGES CXX)

set(GIT_SHA "unknown")
find_program(GIT_EXECUTABLE git)

if (GIT_EXECUTABLE)
    execute_process(COMMAND ${GIT_EXECUTABLE} rev-parse --short=12 HEAD
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_SHA OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET)
    if(NOT GIT_SHA)
        set(GIT_SHA "unknown")
    endif()
endif()

set(ACR_VERSION_HDR ${CMAKE_CURRENT_BINARY_DIR}/generated/ictk/tools/acr/version.hpp)
configure_file(
    ${CMAKE_CURRENT_LIST_DIR}/include_template/version.hpp.in
    ${ACR_VERSION_HDR}
    @ONLY
)

add_library(ictk_acr STATIC 
    src/io/mcap_reader.cpp 
    src/io/sidecar_semantics.cpp
    src/io/events_jsonl_reader.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../evidence_recorder/src/hash.cpp         # # use BLAKE 3 wrapper 
)

add_library(ictk_acr::ictk_acr ALIAS ictk_acr)
set_target_properties(ictk_acr PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(ictk_acr
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/generated>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/src
        ${CMAKE_CURRENT_LIST_DIR}/../evidence_recorder/src
        ${CMAKE_CURRENT_LIST_DIR}/../evidence_recorder/schemas
)

install(FILES ${ACR_VERSION_HDR} DESTINATION include/ictk/tools/acr)

target_compile_features(ictk_acr PUBLIC cxx_std_20)

## compiler options and sanitizer
ictk_apply_compiler_options(ictk_acr)
ictk_apply_sanitizers(ictk_acr)

# # MCAP: reuse the mcap target defined by tools/evidence_recorder
if (TARGET mcap)  
    target_link_libraries(ictk_acr PRIVATE mcap)
else()
    message(FATAL_ERROR "TARGET mcap not found. Configure with -DICTK_RECORDER_BACKEND_MCAP=ON (and ICTK_FETCH_DEPS=ON or system headers).")
endif()


add_executable(acr cli/acr_main.cpp)
target_link_libraries(acr PRIVATE ictk_acr)

install(TARGETS ictk_acr acr
    RUNTIME DESTINATION bin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/include/ DESTINATION include)