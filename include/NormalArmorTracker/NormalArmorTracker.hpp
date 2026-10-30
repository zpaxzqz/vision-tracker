/*
 * @FilePath: include/NormalArmorTracker/NormalArmorTracker.hpp
 * @Description: 普通装甲目标：连续丢失 3 帧判定 LOST，只按 x/y 速度预测位置，无角度。
 */

#pragma once

#include "TrackingBase/TrackingBase.hpp"

namespace rm
{

    class NormalArmorTracker : public TrackingBase
    {
    public:
        NormalArmorTracker(int id, const Observation &obs, std::int64_t timestamp_ns);

        /// 连续未观测 3 帧 → LOST
        int lostThreshold() const override { return 3; }

    private:
        /// 普通装甲不维护角度：angle / angular_velocity / predicted_angle 恒为 0
        bool tracksAngle() const override { return false; }
    };

} // namespace rm
