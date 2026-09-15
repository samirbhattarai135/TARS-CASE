#ifndef CASE_SIM_TEST_VENDOR__WPROGRAM_H_
#define CASE_SIM_TEST_VENDOR__WPROGRAM_H_

// PID_v1.cpp picks this header whenever ARDUINO is undefined, which it always
// is in a host build. Redirecting here keeps the vendored source byte-identical
// to upstream instead of patching its include guard.
#include "Arduino.h"

#endif  // CASE_SIM_TEST_VENDOR__WPROGRAM_H_
