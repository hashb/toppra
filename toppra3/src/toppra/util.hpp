#ifndef UTIL_HPP
#define UTIL_HPP

#include <iostream>

#ifndef TOPT_DEBUG_PRINT
#define TOPT_DEBUG_PRINT 0
#endif

#define TOPT_DEBUG_MSG(msg) \
  do {                      \
    if (TOPT_DEBUG_PRINT) { \
      std::cout << msg;     \
    }                       \
  } while (0)

#endif
