## Build all files in PID

file(GLOB SRCS CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
set(OUTDIR "${CMAKE_BINARY_DIR}/examples/pid")

foreach(src IN LISTS SRCS)
    get_filename_component(name "${src}" NAME_WE)

    set(tgt "example_pid_${name}")

    add_executable(${tgt} "${src}")

    target_link_libraries(${tgt} PRIVATE ictk_core)

    ictk_apply_compiler_options(${tgt})

    set_target_properties(${tgt} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${OUTDIR}")

    add_dependencies(examples_all ${tgt})
endforeach()