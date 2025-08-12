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
        /EHsc
    ) 
    # LTO ib MSVC