function (ictk_apply_compiler_options tgt)
  target_compile_definitions(${tgt} INTERFACE
  $<$<BOOL:${ICTK_NO_EXCEPTIONS}>:ICTK_NO_EXCEPTIONS=1>
  $<$<BOOL:${ICTK_NO_RTTI}>:ICTK_NO_RTTI=1>)

  if (MSVC)
    target_compile_options(${tgt} INTERFACE
        /permissive-
        /W4 /WX
        /Zc:preprocessor /Zc:__cplusplus
        $<$<BOOL:${ICTK_NO_RTTI}>: /GR->
        # MSVC dont truly disable EH, so we enforce "no throw" in code when NO_EXCEPTIONS = 1
        /EHsc
    ) 
    # LTO in MSVC
    target_compile_options(${tgt} INTERFACE
      $<$AND:$<CONFIG:Release>,$<BOOL:${ICTK_ENABLE_LTO}>>:/GL>
    ) 

    target_link_options(${tgt} INTERFACE
      $<$<AND:$<CONFIG:Release>,$<BOOL:${ICTK_ENABLE_LTO}>>:/LTCG>
    )
  
  else()
    # GCC/Clang
    target_compile_options(${tgt} INTERFACE
      -Wall -Wextra -Wpedantic -Werror
      -Wconversion -Wdouble-promotion -Wshadow -Wnon-virtual-dtor -Wold-style-cast
      -ffp-contract=off -fno-math-errno -fno-signed-zeros
      $<$<BOOL:${ICTK_NO_RTTI}>:-fno-rtti>
      $<$<BOOL:${ICTK_NO_EXCEPTIONS}>:-fno-exceptions>
    )

    # LTO for release when requested
    target_compile_options(${tgt} INTERFACE
      $<$<AND:$<CONFIG:Release>,$<BOOL:${ICTK_ENABLE_LTO}>>:-flto>
    )
  
  endif()


  ## Gen a compile_commands.json for tooling

  set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE BOOL "Export compile commands for tooling" FORCE)

endfunction()