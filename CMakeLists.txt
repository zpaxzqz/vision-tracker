cmake_minimum_required(VERSION 3.16)

project(vision_tracker LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

## 给 clangd / VSCode IntelliSense 生成 compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(tracker
  src/main.cpp
  include/data_types.hpp
  include/test_frames.hpp
  include/normal_armor_tracker.hpp
  include/rotate_armor_tracker.hpp
  include/target_manager.hpp
  include/tracker_base.hpp
)

target_include_directories(tracker PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include")
