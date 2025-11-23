cmake_minimum_required(VERSION 3.16)
project(SiliconTwin VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(SILICONTWIN_BUILD_TESTS "Build the Google Test suites" ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
  set(SILICONTWIN_WARNINGS -Wall -Wextra -Wshadow -Wconversion -Wno-sign-conversion)
endif()

# ---------------------------------------------------------------------------
# Firmware under test. Built as an object library that can only see its own
# headers, which enforces that it talks to hardware exclusively through hal.h.
# ---------------------------------------------------------------------------
add_library(silicontwin_firmware OBJECT
  firmware/src/params.c
  firmware/src/retained.c
  firmware/src/control.c
  firmware/src/protection.c
  firmware/src/can_proto.c
  firmware/src/shell.c
  firmware/src/firmware.c
)
target_include_directories(silicontwin_firmware PUBLIC firmware/include)
target_compile_options(silicontwin_firmware PRIVATE ${SILICONTWIN_WARNINGS})

# ---------------------------------------------------------------------------
# Simulator core: virtual clock, peripherals, plant model
# ---------------------------------------------------------------------------
add_library(silicontwin_sim STATIC
  sim/src/clock.c
  sim/src/rng.c
  sim/src/gpio.c
  sim/src/uart.c
  sim/src/can.c
  sim/src/adc.c
  sim/src/pwm.c
  sim/src/watchdog.c
  sim/src/plant.c
  sim/src/mcu.c
  sim/src/hal_sim.c
  sim/src/faults.c
  sim/src/board.c
  sim/src/config.c
  $<TARGET_OBJECTS:silicontwin_firmware>
)
target_include_directories(silicontwin_sim PUBLIC sim/include firmware/include)
target_compile_options(silicontwin_sim PRIVATE ${SILICONTWIN_WARNINGS})
if(NOT MSVC)
  target_link_libraries(silicontwin_sim PUBLIC m)
endif()
