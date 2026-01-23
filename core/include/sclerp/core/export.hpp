#pragma once

#if defined(_WIN32) && defined(SCLERP_CORE_SHARED)
  #if defined(SCLERP_CORE_BUILDING)
    #define SCLERP_CORE_API __declspec(dllexport)
  #else
    #define SCLERP_CORE_API __declspec(dllimport)
  #endif
#else
  #define SCLERP_CORE_API
#endif
