/*
 * @FilePath: src/TargetManager/TargetManager.cpp
 * @Description: 目标管理器实现：整帧校验 → 更新最近成功帧 → 逐观测更新/创建 → 未观测目标推进 → 统计。
 */

#include "TargetManager/TargetManager.hpp"

#include <algorithm>
#include <unordered_map>
#include <unordered_set>

#include "NormalArmorTracker/NormalArmorTracker.hpp"
#include "RotateArmorTracker/RotateArmorTracker.hpp"

namespace rm
{

    namespace
    {
        constexpr double kNsPerSecond = 1'000'000'000.0;
    }

    bool TargetManager::validate(const InputFrame &frame) const
    {
        // 1) 帧号与时间戳都必须严格大于最近一次成功处理的帧
        if (has_processed_frame_)
        {
            if (frame.frame_id <= last_frame_id_)
            {
                return false;
            }
            if (frame.timestamp_ns <= last_timestamp_ns_)
            {
                return false;
            }
        }

        // 2) 观测校验：负数编号、同帧重复、类型与已创建目标不一致
        std::unordered_set<int> seen_ids;
        std::unordered_map<int, TargetType> existing_types;
        for (const auto &target : targets_)
        {
            existing_types.emplace(target->id(), target->type());
        }

        for (const auto &obs : frame.observations)
        {
            if (obs.id < 0)
            {
                return false;
            }
            // insert 返回 false 说明本帧已经出现过同一编号
            if (!seen_ids.insert(obs.id).second)
            {
                return false;
            }
            const auto it = existing_types.find(obs.id);
            if (it != existing_types.end() && it->second != obs.type)
            {
                return false;
            }
        }
        return true;
    }

    void TargetManager::createTarget(const Observation &obs, std::int64_t timestamp_ns)
    {
        if (obs.type == TargetType::SPINNING)
        {
            targets_.push_back(std::make_shared<RotateArmorTracker>(obs.id, obs, timestamp_ns));
        }
        else
        {
            targets_.push_back(std::make_shared<NormalArmorTracker>(obs.id, obs, timestamp_ns));
        }
    }

    Result TargetManager::onFrame(const InputFrame &frame)
    {
        // 任一校验失败：整帧拒绝，返回当前快照，且不推进最近成功帧
        if (!validate(frame))
        {
            return buildResult(frame.frame_id, false);
        }

        // 首帧的时间差取 0；其余按纳秒时间戳换算成秒
        double dt = 0.0;
        if (has_processed_frame_)
        {
            dt = static_cast<double>(frame.timestamp_ns - last_timestamp_ns_) / kNsPerSecond;
        }

        // 按编号索引现有目标，避免每帧线性搜索
        std::unordered_map<int, std::shared_ptr<TrackingBase>> by_id;
        for (const auto &target : targets_)
        {
            by_id.emplace(target->id(), target);
        }

        // 本帧已被观测到的编号；同时用于"新建目标"判定
        std::unordered_set<int> observed_ids;

        // 1) 遍历本帧观测：已创建目标更新，未创建则新建
        for (const auto &obs : frame.observations)
        {
            observed_ids.insert(obs.id);

            auto it = by_id.find(obs.id);
            if (it == by_id.end())
            {
                createTarget(obs, frame.timestamp_ns);
                continue;
            }

            const std::shared_ptr<TrackingBase> &target = it->second;
            // 目标距上次观测的时间差，用于计算速度而非帧间隔
            const double dt_obs =
                static_cast<double>(frame.timestamp_ns - target->lastObservationNs()) / kNsPerSecond;
            target->update(obs, dt, dt_obs, frame.timestamp_ns);
        }

        // 2) 遍历本帧未被观测的目标：丢失计数与预测推进
        for (const auto &target : targets_)
        {
            if (observed_ids.find(target->id()) == observed_ids.end())
            {
                target->onMissed(dt);
            }
        }

        // 3) 本帧成功处理，推进最近成功帧记录
        last_frame_id_ = frame.frame_id;
        last_timestamp_ns_ = frame.timestamp_ns;
        has_processed_frame_ = true;

        return buildResult(frame.frame_id, true);
    }

    Result TargetManager::buildResult(int frame_id, bool accepted) const
    {
        Result result{};
        result.accepted = accepted;
        result.frame_id = frame_id;
        result.targets.reserve(targets_.size());

        for (const auto &target : targets_)
        {
            const TargetSnapshot snap = target->snapshot();
            result.targets.push_back(snap);

            if (snap.status == TargetStatus::TRACK)
            {
                ++result.tracking_target_count;
            }
            else
            {
                ++result.lost_target_count;
            }

            if (snap.type == TargetType::ARMOR)
            {
                ++result.armor_target_count;
            }
            else
            {
                ++result.spinning_target_count;
            }
        }
        return result;
    }

} // namespace rm
