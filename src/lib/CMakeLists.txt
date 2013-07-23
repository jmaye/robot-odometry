remake_include()

remake_add_directories(generic)
remake_add_directories(utils)
remake_pkg_config_generate()

remake_add_directories(janeth COMPONENT janeth)
remake_pkg_config_generate(COMPONENT janeth)
