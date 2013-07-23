remake_find_package(GTest REQUIRED)

remake_include(${GTEST_INCLUDE_DIRS})

remake_add_directories()

remake_add_executables(TESTING LINK generic-test janeth-test
  ${GTEST_MAIN_LIBRARY})
