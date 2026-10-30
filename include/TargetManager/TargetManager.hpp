/*
 * @FilePath: include/TargetManager/TargetManager.hpp
 * @Description: 目标管理器。逐帧接收检测结果，负责整帧校验、逐帧更新、统计与输出快照。
 *               主程序只依赖 onFrame(const InputFrame&) 这一个接口。
 */

#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <vector>

#include "data_types.hpp"
#include "TrackingBase/TrackingBase.hpp"

namespace rm
{

    class TargetManager
    {
    public:
        /// 处理一帧。内部始终维护"最近一次成功帧"，非法帧整帧拒绝且不改变任何状态
        Result onFrame(const InputFrame &frame);

    private:
        /// 整帧校验：帧号/时间戳必须严格递增，观测不得含负数编号、同帧重复编号或类型不一致
        bool validate(const InputFrame &frame) const;

        /// 按观测创建新目标（类型取本次观测）
        void createTarget(const Observation &obs, std::int64_t timestamp_ns);

        /// 构造结果快照：包含全部目标（含 LOST），并统计四类计数
        Result buildResult(int frame_id, bool accepted) const;

        std::vector<std::shared_ptr<TrackingBase>> targets_;

        bool has_processed_frame_ = false;
        int last_frame_id_ = 0;
        std::int64_t last_timestamp_ns_ = 0;
    };

} // namespace rm
