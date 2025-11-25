include(FetchContent)

find_package(GTest QUIET)
if(NOT GTest_FOUND)
  FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.tar.gz
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  )
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)
endif()

add_executable(silicontwin_tests
  test_peripherals.cpp
  test_can.cpp
  test_plant.cpp
  test_firmware_units.cpp
  test_config_scenario.cpp
  test_hil.cpp
)
target_link_libraries(silicontwin_tests PRIVATE silicontwin_sim GTest::gtest_main)
target_compile_definitions(silicontwin_tests PRIVATE
  SILICONTWIN_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

include(GoogleTest)
gtest_discover_tests(silicontwin_tests DISCOVERY_TIMEOUT 60)
