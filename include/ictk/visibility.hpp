#pragma once

#if defined(_WIN32) || defined(_WIN64)

  #if defined(ICTK_BUILD_DLL)
    #define ICTK_API __declspec(dllexport)

  #else
    #define ICTK_API __declspec(dllimport)
    
  #endif

#else
  #define ICTK_API __attribute__((visibility("default")))

#endif