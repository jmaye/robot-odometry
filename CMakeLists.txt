cmake_minimum_required(VERSION 2.6)

include(ReMake)

remake_project(
  robot-odometry
  VERSION 0.1
  RELEASE alpha
  SUMMARY "A free/open source C++ library for robot odometry."
  AUTHOR "Jerome Maye"
  CONTACT "jerome.maye@mavt.ethz.ch"
  HOME "https://github.com/jmaye/robot-odometry"
  LICENSE "GNU Lesser General Public License (LGPL)"
  PREFIX robot-odometry-
  INSTALL /usr/
  EXTRA_CXX_FLAGS -std=c++0x
)

remake_doc(
  html
  latex
)

remake_pack_deb(
  DESCRIPTION "Main library."
)

remake_pack_deb(
  COMPONENT dev
  DESCRIPTION "Development headers."
  DEPENDS robot-odometry libeigen3-dev
)

remake_pack_deb(
  COMPONENT janeth
  DESCRIPTION "JanETH specific odometry."
  DEPENDS robot-odometry
)

remake_pack_deb(
  COMPONENT janeth-dev
  DESCRIPTION "JanETH specific odometry development headers."
  DEPENDS robot-odometry-janeth robot-odometry-dev
)

remake_pack_deb(
  COMPONENT simulation
  DESCRIPTION "Odometry simulation binaries."
  DEPENDS robot-odometry-janeth
)

remake_pack_deb(
  COMPONENT doc
  DESCRIPTION "Documentation"
  DEPENDS robot-odometry
)

remake_distribute_deb(
  DISTRIBUTION lucid
  SECTION libs
  UPLOAD ppa:jmaye/ethz
  DEPENDS remake doxygen pkg-config libeigen3-dev
  PASS CMAKE_BUILD_TYPE ROBOT_ODOMETRY_GIT_REVISION
)

remake_distribute_deb(
  DISTRIBUTION precise
  SECTION libs
  UPLOAD ppa:jmaye/ethz
  DEPENDS remake doxygen pkg-config libeigen3-dev
  PASS CMAKE_BUILD_TYPE ROBOT_ODOMETRY_GIT_REVISION
)

remake_distribute_deb(
  DISTRIBUTION trusty
  SECTION libs
  UPLOAD ppa:jmaye/ethz
  DEPENDS remake doxygen pkg-config libeigen3-dev
  PASS CMAKE_BUILD_TYPE ROBOT_ODOMETRY_GIT_REVISION
)
