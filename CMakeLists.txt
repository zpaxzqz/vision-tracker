cmake_minimum_required(VERSION 3.16)

project(vision_tracker LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## 给 clangd / VSCode IntelliSense 生成 compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(tracker
  src/main.cpp

  # 跟踪基类
  include/TrackingBase/TrackingBase.hpp
  src/TrackingBase/TrackingBase.cpp

  # 普通装甲：3 帧丢失，按 x/y 速度预测
  include/NormalArmorTracker/NormalArmorTracker.hpp
  src/NormalArmorTracker/NormalArmorTracker.cpp

  # 旋转装甲：5 帧丢失，额外维护角度与角速度
  include/RotateArmorTracker/RotateArmorTracker.hpp
  src/RotateArmorTracker/RotateArmorTracker.cpp

  # 目标管理器：校验、逐帧更新、统计
  include/TargetManager/TargetManager.hpp
  src/TargetManager/TargetManager.cpp

  # 数据契约与测试帧（请勿修改）
  include/data_types.hpp
  include/test_frames.hpp
)

target_include_directories(tracker PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include")
