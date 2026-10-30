/*
 * @FilePath: src/RotateArmorTracker/RotateArmorTracker.cpp
 * @Description: 旋转装甲目标实现：角度取观测值，角速度按观测时间差计算。
 */

#include "RotateArmorTracker/RotateArmorTracker.hpp"

namespace rm
{

    RotateArmorTracker::RotateArmorTracker(int id, const Observation &obs, std::int64_t timestamp_ns)
        : TrackingBase(id, TargetType::SPINNING, timestamp_ns)
    {
        // 首个观测即成为当前状态，角速度清零，预测角等于观测角
        update(obs, 0.0, 0.0, timestamp_ns);
    }

} // namespace rm
