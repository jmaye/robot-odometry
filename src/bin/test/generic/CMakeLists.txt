remake_find_package(GTest REQUIRED)

remake_include(${GTEST_INCLUDE_DIRS})

remake_add_library(generic-test LINK generic ${GTEST_LIBRARY})
