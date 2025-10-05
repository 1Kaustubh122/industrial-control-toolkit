cmake_minimum_required(VERSION 3.20)
project(ictk-acr LANGUAGES CXX)

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
        ${CMAKE_CURRENT_LIST_DIR}/include
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/src
        ${CMAKE_CURRENT_LIST_DIR}/../evidence_recorder/src
        ${CMAKE_CURRENT_LIST_DIR}/../evidence_recorder/schemas
)

target_compile_features(ictk_acr PUBLIC cxx_std_20)

## compiler options and sanitizer
ictk_apply_compiler_options(ictk_acr)
ictk_apply_sanitizers(ictk_acr)

## Prefer system mcap + fallback
find_package(mcap QUIET)
if (mcap_FOUND)
    target_link_libraries(ictk_acr PRIVATE mcap::mcap)
else()
    add_library(ictk_mcap_impl OBJECT
        ${CMAKE_CURRENT_LIST_DIR}/../evidence_recorder/src/thirdparty/mcap_impl.cpp
        )
    target_include_directories(ictk_mcap_impl PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/../evidence_recorder/src/thirdparty
    )
    target_link_libraries(ictk_acr PRIVATE ictk_mcap_impl)
endif()

add_executable(acr cli/acr_main.cpp)
target_link_libraries(acr PRIVATE ictk_acr)

install(TARGETS ictk_acr acr
    RUNTIME DESTINATION bin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/include/ DESTINATION include)