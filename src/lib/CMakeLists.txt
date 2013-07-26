remake_include()

remake_add_directories(generic)
remake_add_directories(utils)
remake_pkg_config_generate(EXTRA_CFLAGS -std=c++0x)

remake_add_directories(janeth COMPONENT janeth)
remake_pkg_config_generate(COMPONENT janeth REQUIRES robot-odometry)
