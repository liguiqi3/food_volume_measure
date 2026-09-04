// Conan::ImportStart
#pragma once
#include <vector>
#include "volume_types.hpp"
// Conan::ImportEnd



namespace vm {



/**
 * @brief [en] End-to-end PCD-IM food volume measurement pipeline.
 * @brief [zh] 端到端的 PCD-IM 食材体积测量流水线。
 * @exporter
 */
class VolumePipeline {
  public:
    VolumePipeline() = default;

    /**
     * @brief [en] Measures food volume in cubic centimeters from empty-oven baseline frames and a food frame.
     * @brief [zh] 从空炉基线帧与食材帧测量食材体积（立方厘米）。
     * @param baseline_frames [en] One or more empty-oven point clouds in `cfg.input_unit`.
     * @param baseline_frames [zh] 一帧或多帧以 `cfg.input_unit` 为单位的空炉点云。
     * @param food_frame [en] The food point cloud in `cfg.input_unit`.
     * @param food_frame [zh] 以 `cfg.input_unit` 为单位的食材点云。
     * @param cfg [en] Measurement configuration.
     * @param cfg [zh] 测量配置。
     * @return [en] A VolumeEstimate whose status indicates success or the first failing stage.
     * @return [zh] 一个 VolumeEstimate，其状态指示成功或第一个失败阶段。
     */
    VolumeEstimate measure(const std::vector<PointCloud>& baseline_frames, const PointCloud& food_frame,
                           const MeasurementConfig& cfg) const;
};



} // namespace vm
