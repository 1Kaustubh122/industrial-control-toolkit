cmake_minimum_required(VERSION 3.22)

## Test utils
add_library(ictk_test_util STATIC
    util/alloc_interposer.cpp
)

target_include_directories(ictk_test_util PUBLIC ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(ictk_test_util PUBLIC ictk_core)

## apply compiler options and sanitizers
ictk_apply_compiler_options(ictk_test_util)
ictk_apply_sanitizers(ictk_test_util)


## headers compiler smoke
add_executable(test_headers_compile unit/test_headers_compile.cpp)
target_link_libraries(test_headers_compile PRIVATE ictk_core)
ictk_apply_compiler_options(test_headers_compile)
ictk_apply_sanitizers(test_headers_compile)

## property-> Determinism runner + no alloc after start
add_executable(test_determinism_runner 
    property/test_determinism_runner.cpp
    util/dummy_controller.hpp
)
target_link_libraries(test_determinism_runner PRIVATE ictk_core ictk_test_util)
add_test(NAME determinism_runner COMMAND test_determinism_runner)
ictk_apply_compiler_options(test_determinism_runner)
ictk_apply_sanitizers(test_determinism_runner)

## Placeholders
add_executable(test_safety_order property/test_safety_order.cpp)
target_link_libraries(test_safety_order PRIVATE ictk_core)
add_test(NAME safety_order_placeholder COMMAND test_safety_order)
ictk_apply_compiler_options(test_safety_order)
ictk_apply_sanitizers(test_safety_order)

add_executable(test_bumpless_placeholder property/test_bumpless_placeholder.cpp)
target_link_libraries(test_bumpless_placeholder PRIVATE ictk_core)
add_test(NAME bumpless_placeholder COMMAND test_bumpless_placeholder)
ictk_apply_compiler_options(test_bumpless_placeholder)
ictk_apply_sanitizers(test_bumpless_placeholder)