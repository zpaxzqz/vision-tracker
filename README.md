# vision-tracker · 视觉目标管理与跟踪系统

> 第二次作业 · 创梦之翼视觉组 2026 招新培训
> 实现：C++17，基于 `include/data_types.hpp` 的数据契约，未修改 `data_types.hpp` / `test_frames.hpp`。

---

## 1. 项目概述

逐帧接收检测结果，维护多个被检测目标的跟踪状态：

| 目标类型 | 丢失阈值 | 预测行为 |
|---|---|---|
| 普通装甲 `ARMOR` | 连续 3 帧未观测 → LOST | 按 x/y 速度预测位置，无角度 |
| 旋转装甲 `SPINNING` | 连续 5 帧未观测 → LOST | 按 x/y 速度预测位置，并按角速度预测角度 |

目标管理器负责整帧校验、目标创建、丢失与重新追踪、统计输出。

---

## 2. 工程结构

```
vision-tracker/
├── CMakeLists.txt
├── README.md                          # 本文件
├── .vscode/c_cpp_properties.json
├── include/
│   ├── data_types.hpp                 # 数据契约（未修改）
│   ├── test_frames.hpp                # 28 帧测试序列（未修改）
│   ├── TrackingBase/TrackingBase.hpp
│   ├── NormalArmorTracker/NormalArmorTracker.hpp
│   ├── RotateArmorTracker/RotateArmorTracker.hpp
│   └── TargetManager/TargetManager.hpp
└── src/
    ├── main.cpp                       # 入口 + 输出函数
    ├── TrackingBase/TrackingBase.cpp
    ├── NormalArmorTracker/NormalArmorTracker.cpp
    ├── RotateArmorTracker/RotateArmorTracker.cpp
    └── TargetManager/TargetManager.cpp
```

声明放 `include/`，实现放 `src/`，`#include` 以 `include/` 为根。

---

## 3. 类设计

### 3.1 继承关系

```
        TrackingBase  (纯虚基类, virtual 析构)
          /        \
 NormalArmorTracker  RotateArmorTracker
          \        /
         TargetManager  (持有 vector<shared_ptr<TrackingBase>>)
```

### 3.2 各部分职责

| 类 | 职责 |
|---|---|
| `TrackingBase` | 保存编号、类型、状态、位置、角度、丢失次数、速度与预测值；实现四种状态转换：首次创建 / 正常跟踪 / 未观测推进 / 进入 LOST |
| `NormalArmorTracker` | `lostThreshold() = 3`，`tracksAngle() = false`（角度恒 0） |
| `RotateArmorTracker` | `lostThreshold() = 5`，`tracksAngle() = true`（维护角度与角速度） |
| `TargetManager` | 整帧校验、目标创建、逐观测更新、未观测推进、统计与快照 |

### 3.3 关键设计点

1. **差异用虚函数表达，而不是复制代码。**
   两个派生类只覆写 `lostThreshold()` 和 `tracksAngle()` 两个纯虚函数，其余全部复用基类。
   如果把丢失阈值写死在 `getSnapshot()` 里，就会出现"普通 3 帧 / 旋转 5 帧"无法分流的问题。

2. **速度用 `dt_obs`，预测用 `dt`。**
   - `dt_obs = (t_ns − 上次观测时间戳)/1e9`，用于算速度；
   - `dt = (t_ns − 上一成功帧)/1e9`，用于算预测。
   因此基类必须记录 `last_obs_ns_`。目标中途丢帧后再被观测到时，两者不相等。

3. **首帧/重新追踪用显式标志判定，不复用状态值。**
   `awaiting_first_observation_` 显式标记"还没拿到第二次观测"。
   注意 **不能** 用"速度是否为 0"判断重新追踪：LOST 时速度已被清零，用速度判断会把 LOST 的正常情况误判成首帧。

4. **关联按编号 `id`，不做最近邻匹配。**
   README 明确"同编号首次出现时创建目标"，类型也在首次创建时确定。用坐标距离匹配会产生重复目标。

5. **整帧拒绝在 `validate()` 内一次性判定，且在任何状态修改之前返回。**
   校验失败直接 `buildResult(frame_id, false)`，不推进 `last_frame_id_` / `last_timestamp_ns_`，保证状态不被部分修改，也保证后续帧仍能正常处理。

6. **LOST 时 `predicted_*` 一并清零。**
   这一点容易被漏掉：只清 `x/y/angle/velocity` 而保留 `predicted_*` 会导致 LOST 目标仍有非零预测值。

---

## 4. 关键语义对照

| 场景 | 处理 |
|---|---|
| 首帧 | `dt = 0`，创建目标，速度 0，预测 = 观测 |
| 正常观测到 TRACK 目标 | 用 `dt_obs` 更新速度 → `predicted_* = 当前 + 速度 × dt` |
| 未观测、未达阈值 | `missed++`；`predicted += 速度 × dt`；旋转装甲 `angle += w × dt`，`predicted_angle = angle + w × dt` |
| 未观测、达到阈值 | 那一帧不再推进，直接 `markLost()` 全清零 |
| LOST 后重新观测 | 不做差值，速度清零，预测值 = 本次观测，`missed = 0` |
| 非法帧 | 整帧拒绝，返回不变快照，不推进帧号/时间戳 |

非法帧（5 类）：帧号不大于最近成功帧、时间戳不大于最近成功帧、负数编号、同帧重复编号、类型与已创建目标不一致。

---

## 5. 构建与运行

```bash
# 使用 CMake
cmake -S . -B build
cmake --build build
./build/tracker

# 或直接用 g++
g++ -std=c++17 -I include src/main.cpp src/*/*.cpp -o tracker
./tracker
```

程序输出 28 帧、每帧 2 行表头 + 每目标 1 行。

---

## 6. 验收结果

已按 README 第 10 节逐项核对，全部通过：

- [x] 帧 0–1：普通目标 1 速度 10.0、预测位置 (3,2)；旋转目标 2 速度 0、预测 (5,0)、角速度 2.0、预测角 0.4
- [x] 帧 2–5：旋转目标角度每 0.1 s 增加 0.2，预测角始终超前 0.2；速度保持 0，位置预测保持 (5,0)
- [x] 帧 6：旋转目标 2 进入 LOST，全部字段清零；目标 1 速度 3.33、预测 (5.33,2)
- [x] 帧 7–8：目标 1 预测推进到 (5.67,2)、(6.00,2)
- [x] 帧 9：目标 1 速度 3.33、预测 (6.33,2)、`missed` 清零
- [x] 帧 12：普通目标 1 连续丢失 3 帧后进入 LOST
- [x] 帧 13、21、22：LOST 目标重新追踪，速度清零，预测值 = 观测值
- [x] 第一次 16 / 第一次 19 / 第二次 15 / 帧 23 / 帧 24：五类非法帧全部 `accepted=false`，状态不变
- [x] 第二次 17：编号 9 首次出现并创建
- [x] 帧 20：1、3、9 一起进入 LOST
- [x] 非法帧之后仍能正常处理 16–22
- [x] 计数与 `targets` 状态、类型一致，且 `armor + spinning == targets.size()`
- [x] LOST 时 `x, y, angle, velocity, angular_velocity, predicted_*` 全部为 0
- [x] 全程无 NaN / inf
