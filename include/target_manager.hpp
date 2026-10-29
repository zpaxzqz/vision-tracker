#pragma once
#include <vector>
#include <memory>
#include <cmath>
#include <algorithm>
#include "data_types.hpp"
#include "tracker_base.hpp"
#include "normal_armor_tracker.hpp"
#include "rotate_armor_tracker.hpp"

namespace rm
{
class TargetManager
{
private:
    std::vector<std::shared_ptr<TrackerBase>> trackers_;
    std::int64_t last_timestamp_ns =0;
public:
    Result onFrame(const InputFrame &frame)
    {
        Result res{};
        res.frame_id = frame.frame_id;
        double dt=0.0;
        if(last_timestamp_ns!=0)
        if (last_timestamp_ns != 0)
        {
            dt = (frame.timestamp_ns - last_timestamp_ns) / 1000000000.0;
        }
        last_timestamp_ns = frame.timestamp_ns;
        for(auto& t :trackers_)
        {
            t->predict(dt);
        }
        std::vector<bool>obs_used(frame.observations.size(),false);
        for(auto& tracker : trackers_)
        {
            double min_dist=1e9;
            int best_idx=-1;
            auto snap=tracker->getSnapshot();
            for(size_t i=0;i<frame.observations.size();i++)
            {
                if(obs_used[i]) continue;
                const auto& ob =frame.observations[i];
                double dx=snap.predicted_x-ob.x;
                double dy=snap.predicted_y-ob.y;
                double dist=sqrt(dx*dx+dy*dy);
                if(dist <0.8&&dist <min_dist)
                {
                    min_dist=dist;
                    best_idx=i;
                }
            }
            if(best_idx>=0)
            {
                tracker->update(frame.observations[best_idx],dt);
                obs_used[best_idx]=true;
            }
            else
            {
                tracker->markMiss();
            }
        }
        for(size_t i=0;i<frame.observations.size();i++)
        {
            if(obs_used[i]) continue;
            const auto& ob=frame.observations[i];
            std::shared_ptr<TrackerBase> new_tracker;
            if(ob.type==TargetType::ARMOR)
            {
                new_tracker=std::make_shared<NormalArmorTracker>(ob.id,ob);
            }
            else{
                new_tracker=std::make_shared<RotateArmorTracker>(ob.id,ob);
            }
            trackers_.push_back(new_tracker);
    
        }
        int tracking_cnt=0,lost_cnt=0,armor_cnt=0,spin_cnt=0;
        res.targets.clear();
        for(auto& t:trackers_)
        {
            auto snap=t->getSnapshot();
            res.targets.push_back(snap);
            if(snap.status==TargetStatus::TRACK) tracking_cnt++;
            else lost_cnt++;
            if(snap.type==TargetType::ARMOR) armor_cnt++;
            else spin_cnt++;
        }    
        res.tracking_target_count =tracking_cnt;
        res.lost_target_count =lost_cnt;
        res.armor_target_count =armor_cnt;
        res.spinning_target_count=spin_cnt;
        res.accepted=true;
        return res;
    };
};
}