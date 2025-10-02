## Build every examples -> *.cpp

# build option
option(ICTK_BUILD_EXAMPLES "Build ICTK examples" ON)
set(ICTK_EXAMPLES "" CACHE STRING "Semicolon list of categories to build (empty=all)")

if(NOT ICTK_BUILD_EXAMPLES)
    return()
endif()

add_custom_target(examples_all)

# add category subdirs that have a CMakeLists.txt
file(GLOB children LIST_DIRECTORIES true "${CMAKE_CURRENT_SOURCE_DIR}/*")
foreach(dir IN LISTS children)
    if(IS_DIRECTORY "${dir}" AND EXISTS "${dir}/CMakeLists.txt")
        get_filename_component(cat "${dir}" NAME)
        if(ICTK_EXAMPLES STREQUAL "" OR ";${ICTK_EXAMPLES};" MATCHES ";${cat};")
            add_subdirectory("${dir}")
        endif()
    endif()
endforeach()