/*
 * @FilePath: src/NormalArmorTracker/NormalArmorTracker.cpp
 * @Description: 普通装甲目标实现：角度固定为 0，其余行为全部复用基类。
 */

#include "NormalArmorTracker/NormalArmorTracker.hpp"

namespace rm
{

    NormalArmorTracker::NormalArmorTracker(int id, const Observation &obs, std::int64_t timestamp_ns)
        : TrackingBase(id, TargetType::ARMOR, timestamp_ns)
    {
        // 首个观测即成为当前位置，速度清零，预测值等于观测值
        update(obs, 0.0, 0.0, timestamp_ns);

        // 普通装甲的 angle 字段无意义，固定为 0（数据契约 4.3）
        setAngle(0.0);
    }

} // namespace rm
