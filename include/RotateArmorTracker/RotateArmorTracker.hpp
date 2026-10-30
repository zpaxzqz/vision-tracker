/*
 * @FilePath: include/RotateArmorTracker/RotateArmorTracker.hpp
 * @Description: 旋转装甲目标：连续丢失 5 帧判定 LOST，除 x/y 速度外还维护角度与角速度。
 */

#pragma once

#include "TrackingBase/TrackingBase.hpp"

namespace rm
{

    class RotateArmorTracker : public TrackingBase
    {
    public:
        RotateArmorTracker(int id, const Observation &obs, std::int64_t timestamp_ns);

        /// 连续未观测 5 帧 → LOST
        int lostThreshold() const override { return 5; }

    private:
        /// 旋转装甲额外维护 angle / angular_velocity / predicted_angle
        bool tracksAngle() const override { return true; }
    };

} // namespace rm
