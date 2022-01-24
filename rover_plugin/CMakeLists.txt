cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(IGN_PLUGIN_VER 0)
ign_find_package(ignition-plugin0 REQUIRED COMPONENTS register)
# Add sources for each plugin to be registered.
add_library(RoverPlugin rover_plugin.cc)
set_property(TARGET RoverPlugin PROPERTY CXX_STANDARD 17)
target_link_libraries(RoverPlugin
  ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
  ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
)
