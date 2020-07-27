#pragma once

#ifndef __has_attribute
  #define __has_attribute(x) 0
#endif
#if (defined(__GNUC__) && ((__GNUC__ > 4) || (__GNUC__ == 4) && (__GNUC_MINOR__ > 2))) || __has_attribute(visibility)
  #define WPIMATHEXPORT __attribute__((visibility("default")))
#elif _WIN32
  #define WPIMATHEXPORT __declspec(dllexport)
#else
  #define WPIMATHEXPORT
#endif
