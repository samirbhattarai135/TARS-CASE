#ifndef CASE_SIM_TEST_VENDOR__ARDUINO_H_
#define CASE_SIM_TEST_VENDOR__ARDUINO_H_

// Minimal Arduino core stub so the vendored PID_v1 compiles and runs on the
// host. PID_v1 touches nothing from the core except millis(), and the test owns
// that clock so the library and the port can be stepped in lockstep.
//
// Defined by the test translation unit.
extern unsigned long g_fake_millis;

inline unsigned long millis() {return g_fake_millis;}

#endif  // CASE_SIM_TEST_VENDOR__ARDUINO_H_
