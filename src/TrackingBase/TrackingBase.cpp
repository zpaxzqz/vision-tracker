/*
 * @FilePath: src/TrackingBase/TrackingBase.cpp
 * @Description: 跟踪基类实现：首次创建 / 正常跟踪 / 未观测推进 / 进入 LOST 四种状态转换。
 */

#include "TrackingBase/TrackingBase.hpp"

#include <cmath>

namespace rm
{

    TrackingBase::TrackingBase(int id, TargetType type, std::int64_t timestamp_ns)
        : id_(id), type_(type), last_obs_ns_(timestamp_ns)
    {
    }

    void TrackingBase::setAngle(double angle)
    {
        angle_ = angle;
        predicted_angle_ = angle;
    }

    void TrackingBase::update(const Observation &obs, double dt, double dtObs, std::int64_t timestamp_ns)
    {
        // 此前从未被观测过，或上一次已丢失：重新追踪，不做差值计算。
        // 注意 LOST 时速度/预测值已被清零，这里不能用"速度是否为 0"来判定首帧。
        if (awaiting_first_observation_ || status_ == TargetStatus::LOST)
        {
            reacquire(obs, timestamp_ns);
            return;
        }
        trackObservation(obs, dt, dtObs);
        last_obs_ns_ = timestamp_ns;
        missed_count_ = 0;
    }

    void TrackingBase::reacquire(const Observation &obs, std::int64_t timestamp_ns)
    {
        x_ = obs.x;
        y_ = obs.y;
        if (tracksAngle())
        {
            angle_ = obs.angle;
        }
        velocity_x_ = 0.0;
        velocity_y_ = 0.0;
        angular_velocity_ = 0.0;
        predicted_x_ = x_;
        predicted_y_ = y_;
        predicted_angle_ = angle_;

        status_ = TargetStatus::TRACK;
        missed_count_ = 0;
        awaiting_first_observation_ = false;
        last_obs_ns_ = timestamp_ns;
    }

    void TrackingBase::trackObservation(const Observation &obs, double dt, double dtObs)
    {
        // 速度用"距上次观测的时间差"，保证目标中途丢帧后仍得到真实速率
        if (dtObs > 0.0)
        {
            velocity_x_ = (obs.x - x_) / dtObs;
            velocity_y_ = (obs.y - y_) / dtObs;
            if (tracksAngle())
            {
                angular_velocity_ = (obs.angle - angle_) / dtObs;
            }
        }

        x_ = obs.x;
        y_ = obs.y;
        if (tracksAngle())
        {
            angle_ = obs.angle;
        }

        // 预测使用"当前帧的帧间隔"，且只写入 predicted_*，不覆盖 x / y
        predicted_x_ = x_ + velocity_x_ * dt;
        predicted_y_ = y_ + velocity_y_ * dt;
        if (tracksAngle())
        {
            predicted_angle_ = angle_ + angular_velocity_ * dt;
        }
    }

    void TrackingBase::onMissed(double dt)
    {
        if (status_ == TargetStatus::LOST)
        {
            // 已 LOST 的目标保持不动，只保留在容器中等待重新追踪
            return;
        }

        ++missed_count_;
        if (missed_count_ >= lostThreshold())
        {
            // 达到阈值的那一帧不再推进预测，直接清零
            markLost();
            return;
        }

        // 未达阈值：在上一帧预测值上继续累加，速度保持最后一次观测值
        predicted_x_ += velocity_x_ * dt;
        predicted_y_ += velocity_y_ * dt;
        if (tracksAngle())
        {
            angle_ += angular_velocity_ * dt;
            predicted_angle_ = angle_ + angular_velocity_ * dt;
        }
    }

    void TrackingBase::markLost()
    {
        status_ = TargetStatus::LOST;
        x_ = 0.0;
        y_ = 0.0;
        angle_ = 0.0;
        velocity_x_ = 0.0;
        velocity_y_ = 0.0;
        angular_velocity_ = 0.0;
        predicted_x_ = 0.0;
        predicted_y_ = 0.0;
        predicted_angle_ = 0.0;
    }

    TargetSnapshot TrackingBase::snapshot() const
    {
        TargetSnapshot snap{};
        snap.id = id_;
        snap.type = type_;
        snap.status = status_;
        snap.missed_count = missed_count_;

        if (status_ == TargetStatus::TRACK)
        {
            snap.x = x_;
            snap.y = y_;
            snap.velocity_x = velocity_x_;
            snap.velocity_y = velocity_y_;
            snap.angle = tracksAngle() ? angle_ : 0.0;
            snap.angular_velocity = tracksAngle() ? angular_velocity_ : 0.0;
            snap.predicted_x = predicted_x_;
            snap.predicted_y = predicted_y_;
            snap.predicted_angle = tracksAngle() ? predicted_angle_ : 0.0;
        }
        else
        {
            // LOST：位置、角度、速度与预测值全部为 0（由 markLost 保证，这里显式给出）
            snap.x = 0.0;
            snap.y = 0.0;
            snap.velocity_x = 0.0;
            snap.velocity_y = 0.0;
            snap.angle = 0.0;
            snap.angular_velocity = 0.0;
            snap.predicted_x = 0.0;
            snap.predicted_y = 0.0;
            snap.predicted_angle = 0.0;
        }
        return snap;
    }

} // namespace rm
