// Conan::ImportStart
#include <cstddef>
#include <limits>
#include <map>
#include <string>
#include <utility>
#include <vector>
#include "volume_pipeline.hpp"
#include "volume_baseline.hpp"
#include "volume_component.hpp"
#include "volume_integrator.hpp"
#include "volume_pointcloudprocess.hpp"
// Conan::ImportEnd



namespace vm {



namespace {

VolumeEstimate failure(MeasurementStatus status, std::string message) {
    VolumeEstimate est{};
    est.status = status;
    est.message = std::move(message);
    return est;
}

// Chooses orientation points from the largest 3D DBSCAN component, or empty when none qualifies.
PointCloud select_largest_cluster(const PointCloud& cloud_m, const MeasurementConfig& cfg) {
#ifdef __ARM_EABI__
    (void)cloud_m;
    (void)cfg;
    return PointCloud{};
#else
    PointCloud out{};
    if (cloud_m.points.empty() || cfg.cluster_eps_m <= 0.0F || cfg.cluster_min_points <= 0) {
        return out;
    }

    const std::vector<int> labels = dbscan_labels(cloud_m.points, cfg.cluster_eps_m, cfg.cluster_min_points);

    std::map<int, std::size_t> counts;
    for (const int label : labels) {
        if (label >= 0) {
            counts[label] += 1;
        }
    }
    if (counts.empty()) {
        return out;
    }

    int best_label = -1;
    std::size_t best_count = 0;
    for (const auto& entry : counts) {
        if (entry.second > best_count) {
            best_label = entry.first;
            best_count = entry.second;
        }
    }

    out.points.reserve(best_count);
    for (std::size_t i = 0; i < labels.size(); ++i) {
        if (labels[i] == best_label) {
            out.points.push_back(cloud_m.points[i]);
        }
    }
    return out;
#endif // __ARM_EABI__
}

} // namespace



/**
 * @brief [en] Measures food volume in cubic centimeters from empty-oven baseline frames and a food frame.
 * @brief [zh] 从空炉基线帧与食材帧测量食材体积（立方厘米）。
 * @attacher
 */
VolumeEstimate VolumePipeline::measure(const std::vector<PointCloud>& baseline_frames, const PointCloud& food_frame,
                                       const MeasurementConfig& cfg) const {
#ifdef __ARM_EABI__
    (void)baseline_frames;
    (void)food_frame;
    (void)cfg;
    return failure(MeasurementStatus::kUnsupportedPlatform, "PCL volume pipeline is unavailable on bare-metal");
#else
    // 1. Preprocess the food frame (unit-normalize, optional crop, voxel downsample).
    PreprocessResult pre{};
    MeasurementStatus st = preprocess_cloud(food_frame, cfg, pre);
    if (st != MeasurementStatus::kSuccess) {
        return failure(st, status_to_string(st));
    }

    // 2. Remove the dominant background plane(s) from the food frame.
    PointCloud remaining{};
    st = remove_dominant_plane(pre.cloud, cfg, remaining);
    if (st != MeasurementStatus::kSuccess) {
        return failure(st, status_to_string(st));
    }

    // 3. Orientation points: largest DBSCAN component, or all remaining when none qualifies.
    PointCloud orientation = select_largest_cluster(remaining, cfg);
    if (orientation.points.empty()) {
        orientation = remaining;
    }

    // 4. Build the reusable empty-oven baseline model.
    BaselineModel baseline{};
    st = build_baseline_model(baseline_frames, orientation, cfg, baseline);
    if (st != MeasurementStatus::kSuccess) {
        return failure(st, status_to_string(st));
    }

    // 5. Extract selected food components.
    FoodComponents components{};
    st = extract_food_components(remaining, baseline, cfg, components);
    if (st != MeasurementStatus::kSuccess) {
        return failure(st, status_to_string(st));
    }

    // 6. Measure the component volume.
    ComponentVolumeEstimate component_volume{};
    st = measure_component_volume(components, baseline, cfg, component_volume);
    if (st != MeasurementStatus::kSuccess) {
        return failure(st, status_to_string(st));
    }

    // 7. Translate the component result and pipeline diagnostics into the public estimate.
    VolumeEstimate est{};
    est.status = MeasurementStatus::kSuccess;
    est.volume_cm3 = component_volume.volume_cm3;
    est.raw_volume_cm3 = component_volume.raw_volume_cm3;
    est.interpolated_volume_cm3 = component_volume.interpolated_volume_cm3;
    est.uncertainty_cm3 = std::numeric_limits<double>::quiet_NaN();
    est.baseline_frames = baseline.frame_count;
    est.baseline_cell_count = baseline.cell_count;
    est.component_count = components.labels.size();
    est.measured_cells = component_volume.measured_cells;
    est.interpolated_cells = component_volume.interpolated_cells;
    est.occupied_cells = component_volume.occupied_cells;
    est.missing_baseline_cells = component_volume.missing_baseline_cells;
    est.unfilled_hole_cells = component_volume.unfilled_hole_cells;
    est.footprint_area_m2 = component_volume.footprint_area_m2;
    est.coverage_ratio = component_volume.coverage_ratio;
    est.mean_height_m = component_volume.mean_height_m;
    est.max_height_m = component_volume.max_height_m;
    est.message = status_to_string(MeasurementStatus::kSuccess);
    return est;
#endif // __ARM_EABI__
}



} // namespace vm
