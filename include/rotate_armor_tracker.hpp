#pragma once
#include "tracker_base.hpp"
#include<cmath>

namespace rm
{
class RotateArmorTracker : public TrackerBase
{
private:
    double x_,y_;
    double vx_,vy_;
    double angle_,w_;
    double pred_x_,pred_y_,pred_angle_;
public:
    RotateArmorTracker(int id,const Observation& obs)
    : TrackerBase(id,TargetType::SPINNING)
    {
        x_=obs.x;
        y_=obs.y;
        vx_=0.0;
        vy_=0.0;
        angle_=obs.angle;
        w_=0.0;
        pred_x_=x_;
        pred_y_=y_;
        pred_angle_=angle_;
    }
    void update(const Observation& obs,double dt) override
    {
        vx_=(obs.x-x_)/dt;
        vy_=(obs.y-y_)/dt;
        x_=obs.x;
        y_=obs.y;
        double delta_angle=obs.angle-angle_;
        w_=delta_angle/dt;
        angle_=obs.angle;
        clearMiss();
    }
    void predict(double dt) override
    {
        pred_x_ = x_ + vx_ * dt;
        pred_y_ = y_ + vy_ * dt;
        pred_angle_ = angle_ + w_ * dt;
    }
    TargetSnapshot getSnapshot() const override
    {
        TargetSnapshot snap{};
        snap.id = id_;
        snap.type = type_;
        snap.status = (missed_count_ > 5) ? TargetStatus::LOST : TargetStatus::TRACK;

        if (snap.status == TargetStatus::LOST)
        {
            snap.x = 0;
            snap.y = 0;
            snap.angle = 0;
            snap.velocity_x = 0;
            snap.velocity_y = 0;
            snap.angular_velocity = 0;
        }
        else
        {
            snap.x = x_;
            snap.y = y_;
            snap.velocity_x = vx_;
            snap.velocity_y = vy_;
            snap.angle = angle_;
            snap.angular_velocity = w_;
        }
        snap.predicted_x = pred_x_;
        snap.predicted_y = pred_y_;
        snap.predicted_angle = pred_angle_;
        snap.missed_count = missed_count_;
        return snap;
    }
};
}