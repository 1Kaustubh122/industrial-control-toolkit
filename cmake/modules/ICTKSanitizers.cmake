## Applying sanitizers
function(ictk_apply_sanitizers tgt)  
    ## Sanitizers only for debug builds not for release builds
    if (NOT CMAKE_BUILD_TYPE MATCHES "Debug")
        return()
    endif()

    ## MSVC: skip Asan/UBsan support is limited/variant-dependent
    if (MSVC)
        return()
    endif()

    ## AddressSanitizer
    if (ICTK_ENABLE_ASAN)
        target_compile_options(${tgt} INTERFACE
            -fsanitize=address
            -fno-omit-frame-pointer
        )

        target_link_options(${tgt} INTERFACE
            -fsanitize=address
        )
    endif()

    ## UndefinedBehaviorSanitizer
    if (ICTK_ENABLE_UBSAN)
        target_compile_options(${tgt} INTERFACE
            -fsanitize=undefined
            -fno-omit-frame-pointer
        )

        target_link_options(${tgt} INTERFACE
            -fsanitize=undefined
        )
    endif()

endfunction()
