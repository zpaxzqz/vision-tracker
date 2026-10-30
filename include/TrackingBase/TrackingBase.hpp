/*
 * @FilePath: include/TrackingBase/TrackingBase.hpp
 * @Description: 跟踪基类。保存编号、类型、位置、状态、角度、丢失次数、速度与预测值，
 *               提供两个派生类共用的状态更新与快照逻辑；
 *               差异（丢失阈值、是否维护角度）由纯虚函数交给派生类决定。
 */

#pragma once

#include <cstdint>

#include "data_types.hpp"

namespace rm
{

    class TrackingBase
    {
    public:
        /// 编号与类型在首次创建时确定，之后不再改变
        TrackingBase(int id, TargetType type, std::int64_t timestamp_ns);
        virtual ~TrackingBase() = default;

        // 基类指针独占派生对象，禁止拷贝，避免编号/类型被复制出歧义
        TrackingBase(const TrackingBase &) = delete;
        TrackingBase &operator=(const TrackingBase &) = delete;

        /// 连续丢失多少帧后判定 LOST：普通装甲 3，旋转装甲 5
        virtual int lostThreshold() const = 0;

        /// 用本帧观测更新目标。dt 为当前帧间隔，dtObs 为距上次观测的时间差
        void update(const Observation &obs, double dt, double dtObs, std::int64_t timestamp_ns);

        /// 本帧未观测到、且仍为 TRACK 时调用：丢失次数 +1，未达阈值则继续推进预测
        void onMissed(double dt);

        /// 生成快照；LOST 时全部状态与预测值清零
        TargetSnapshot snapshot() const;

        int id() const { return id_; }
        TargetType type() const { return type_; }
        TargetStatus status() const { return status_; }
        bool isLost() const { return status_ == TargetStatus::LOST; }
        int missedCount() const { return missed_count_; }

        /// 上次被观测到时的时间戳，用于计算 dt_obs
        std::int64_t lastObservationNs() const { return last_obs_ns_; }

    protected:
        /// 派生类构造时写入自己的角度初值（普通装甲固定 0，旋转装甲取观测角）
        void setAngle(double angle);

    private:
        /// 是否维护角度与角速度：旋转装甲 true，普通装甲 false
        virtual bool tracksAngle() const = 0;

        /// 由 LOST 重新追踪：不做差值计算，速度清零，预测值取本次观测
        void reacquire(const Observation &obs, std::int64_t timestamp_ns);

        /// TRACK 状态下按观测差更新速度，再按帧间隔计算预测值
        void trackObservation(const Observation &obs, double dt, double dtObs);

        /// 进入 LOST：位置、角度、速度与预测值全部清零
        void markLost();

        int id_;
        TargetType type_;
        TargetStatus status_ = TargetStatus::TRACK;
        int missed_count_ = 0;

        // 尚未拿到"第二次"观测：首帧没有参照，不能做差值
        bool awaiting_first_observation_ = true;

        std::int64_t last_obs_ns_ = 0; // 上次观测时间戳，用于 dt_obs

        double x_ = 0.0;
        double y_ = 0.0;
        double angle_ = 0.0;
        double velocity_x_ = 0.0;
        double velocity_y_ = 0.0;
        double angular_velocity_ = 0.0;
        double predicted_x_ = 0.0;
        double predicted_y_ = 0.0;
        double predicted_angle_ = 0.0;
    };

} // namespace rm
