remake_find_package(GTest REQUIRED)

remake_include(${GTEST_INCLUDE_DIRS})

remake_add_library(janeth-test LINK janeth ${GTEST_LIBRARY})
