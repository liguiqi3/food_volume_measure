#include <iostream>
#include <vector>
#include "volume_pointcloudprocess.hpp"
#include "volume_pipeline.hpp"



namespace {

vm::PointCloud make_baseline(double half = 0.15) {
    vm::PointCloud cloud;
    for (float x = static_cast<float>(-half); x <= static_cast<float>(half) + 1.0e-6F; x += 0.005F) {
        for (float y = static_cast<float>(-half); y <= static_cast<float>(half) + 1.0e-6F; y += 0.005F) {
            cloud.points.push_back({x, y, 0.0F});
        }
    }
    return cloud;
}

vm::PointCloud make_food() {
    vm::PointCloud cloud = make_baseline();
    for (float x = 0.0025F; x <= 0.0975F + 1.0e-6F; x += 0.005F) {
        for (float y = 0.0025F; y <= 0.0975F + 1.0e-6F; y += 0.005F) {
            cloud.points.push_back({x, y, 0.02F});
        }
    }
    return cloud;
}

vm::MeasurementConfig make_config() {
    vm::MeasurementConfig cfg;
    cfg.voxel_size_m = 0.002F;
    cfg.plane_distance_threshold_m = 0.003F;
    cfg.integration_resolution_m = 0.005F;
    cfg.roi_border_margin_m = 0.02F;
    cfg.min_height_m = 0.0015F;
    cfg.baseline_fill_radius_cells = 2;
    cfg.cluster_min_points = 8;
    cfg.foreground_cluster_eps_m = 0.010F;
    cfg.cluster_eps_m = 0.02F;
    return cfg;
}

} // namespace



int main() {
#ifndef __ARM_EABI__
    const std::vector<vm::PointCloud> baselines{make_baseline()};
    vm::VolumePipeline pipeline;
    const auto est = pipeline.measure(baselines, make_food(), make_config());
    std::cout << "status: " << vm::status_to_string(est.status) << std::endl;
    std::cout << "volume_cm3: " << est.volume_cm3 << ", raw_cm3: " << est.raw_volume_cm3
              << ", interpolated_cm3: " << est.interpolated_volume_cm3 << std::endl;
    std::cout << "components: " << est.component_count << ", measured_cells: " << est.measured_cells
              << ", interpolated_cells: " << est.interpolated_cells << std::endl;

    // Demonstrate the public DBSCAN clustering algorithm (Open3D-equivalent labels).
    const std::vector<vm::Point3f> points{{0.0F, 0.0F, 0.0F}, {0.001F, 0.0F, 0.0F}, {0.1F, 0.1F, 0.1F}};
    const std::vector<int> labels = vm::dbscan_labels(points, 0.01, 2);
    std::cout << "dbscan labels:";
    for (const int label : labels) {
        std::cout << ' ' << label;
    }
    std::cout << std::endl;

    return est.status == vm::MeasurementStatus::kSuccess ? 0 : 1;
#else
    std::cout << "volume measurement unavailable on bare-metal" << std::endl;
    return 0;
#endif
}
