// Conan::ImportStart
#include "volume_c.h"
#include <cstddef>
#include <string>
#include <utility>
#include <vector>
#include "volume_baseline.hpp"
#include "volume_component.hpp"
#include "volume_integrator.hpp"
#include "volume_log.hpp"
#include "volume_pipeline.hpp"
#include "volume_pointcloudprocess.hpp"
#include "volume_types.hpp"
// Conan::ImportEnd



namespace {



vm_status_t to_c(vm::MeasurementStatus status) { return static_cast<vm_status_t>(status); }

vm::MeasurementStatus from_c(vm_status_t status) {
    return static_cast<vm::MeasurementStatus>(static_cast<int>(status));
}

std::vector<vm::Point3f> to_cpp_points(const vm_cloud_t* cloud) {
    std::vector<vm::Point3f> out;
    if (cloud == nullptr) {
        return out;
    }
    out.reserve(cloud->count);
    for (size_t i = 0; i < cloud->count; ++i) {
        out.push_back(vm::Point3f{cloud->points[i].x, cloud->points[i].y, cloud->points[i].z});
    }
    return out;
}

vm::PointCloud to_cpp_cloud(const vm_cloud_t* cloud) {
    vm::PointCloud out;
    out.points = to_cpp_points(cloud);
    return out;
}

std::vector<vm::PointCloud> to_cpp_clouds(const vm_cloud_t* frames, size_t count) {
    std::vector<vm::PointCloud> out;
    if (frames == nullptr) {
        return out;
    }
    out.reserve(count);
    for (size_t i = 0; i < count; ++i) {
        out.push_back(to_cpp_cloud(&frames[i]));
    }
    return out;
}

vm::MeasurementConfig to_cpp_config(const vm_config_t* cfg) {
    vm::MeasurementConfig out;
    if (cfg == nullptr) {
        return out;
    }
    out.input_unit = static_cast<vm::LengthUnit>(cfg->input_unit);
    out.use_roi = cfg->use_roi != 0;
    out.roi.min_x = cfg->roi.min_x;
    out.roi.max_x = cfg->roi.max_x;
    out.roi.min_y = cfg->roi.min_y;
    out.roi.max_y = cfg->roi.max_y;
    out.roi.min_z = cfg->roi.min_z;
    out.roi.max_z = cfg->roi.max_z;
    out.voxel_size_m = cfg->voxel_size_m;
    out.plane_distance_threshold_m = cfg->plane_distance_threshold_m;
    out.plane_ransac_iterations = cfg->plane_ransac_iterations;
    out.baseline_max_surface_height_m = cfg->baseline_max_surface_height_m;
    out.remove_secondary_plane = cfg->remove_secondary_plane != 0;
    out.secondary_plane_distance_threshold_m = cfg->secondary_plane_distance_threshold_m;
    out.cluster_eps_m = cfg->cluster_eps_m;
    out.foreground_cluster_eps_m = cfg->foreground_cluster_eps_m;
    out.cluster_min_points = cfg->cluster_min_points;
    out.selection_mode = static_cast<vm::ComponentSelectionMode>(cfg->selection_mode);
    if (cfg->selected_labels != nullptr && cfg->selected_label_count > 0) {
        out.selected_labels.assign(cfg->selected_labels, cfg->selected_labels + cfg->selected_label_count);
    }
    out.integration_resolution_m = cfg->integration_resolution_m;
    out.roi_border_margin_m = cfg->roi_border_margin_m;
    out.min_height_m = cfg->min_height_m;
    out.max_height_m = cfg->max_height_m;
    out.baseline_fill_radius_cells = cfg->baseline_fill_radius_cells;
    out.hole_fill_max_cells = cfg->hole_fill_max_cells;
    out.hole_fill_neighbor_radius_cells = cfg->hole_fill_neighbor_radius_cells;
    out.hole_fill_max_neighbor_height_delta_m = cfg->hole_fill_max_neighbor_height_delta_m;
    out.curve_fill_max_hole_area_cm2 = cfg->curve_fill_max_hole_area_cm2;
    out.curve_fill_max_component_area_ratio = cfg->curve_fill_max_component_area_ratio;
    out.curve_fill_max_imputed_ratio = cfg->curve_fill_max_imputed_ratio;
    out.curve_fill_rim_radius_cells = cfg->curve_fill_rim_radius_cells;
    out.curve_fill_min_rim_samples = cfg->curve_fill_min_rim_samples;
    out.curve_fill_min_rim_coverage = cfg->curve_fill_min_rim_coverage;
    out.curve_fill_max_fit_rmse_m = cfg->curve_fill_max_fit_rmse_m;
    out.curve_fill_max_prediction_rise_m = cfg->curve_fill_max_prediction_rise_m;
    return out;
}

void fill_component_volume(const vm::ComponentVolumeEstimate& in, vm_component_volume_t* out) {
    out->raw_volume_cm3 = in.raw_volume_cm3;
    out->interpolated_volume_cm3 = in.interpolated_volume_cm3;
    out->volume_cm3 = in.volume_cm3;
    out->top_surface_points = in.top_surface_points;
    out->measured_cells = in.measured_cells;
    out->interpolated_cells = in.interpolated_cells;
    out->occupied_cells = in.occupied_cells;
    out->bbox_cell_count = in.bbox_cell_count;
    out->missing_baseline_cells = in.missing_baseline_cells;
    out->unfilled_hole_cells = in.unfilled_hole_cells;
    out->footprint_area_m2 = in.footprint_area_m2;
    out->coverage_ratio = in.coverage_ratio;
    out->mean_height_m = in.mean_height_m;
    out->max_height_m = in.max_height_m;
}

void fill_estimate(const vm::VolumeEstimate& in, vm_estimate_t* out) {
    out->status = to_c(in.status);
    out->volume_cm3 = in.volume_cm3;
    out->raw_volume_cm3 = in.raw_volume_cm3;
    out->interpolated_volume_cm3 = in.interpolated_volume_cm3;
    out->uncertainty_cm3 = in.uncertainty_cm3;
    out->input_points = in.input_points;
    out->downsampled_points = in.downsampled_points;
    out->cluster_count = in.cluster_count;
    const size_t label_count =
        in.selected_cluster_labels.size() < VM_MAX_COMPONENTS ? in.selected_cluster_labels.size() : VM_MAX_COMPONENTS;
    out->selected_cluster_count = label_count;
    for (size_t i = 0; i < label_count; ++i) {
        out->selected_cluster_labels[i] = in.selected_cluster_labels[i];
    }
    out->selected_cluster_points = in.selected_cluster_points;
    out->baseline_frames = in.baseline_frames;
    out->baseline_cell_count = in.baseline_cell_count;
    out->component_count = in.component_count;
    out->top_surface_points = in.top_surface_points;
    out->measured_cells = in.measured_cells;
    out->interpolated_cells = in.interpolated_cells;
    out->occupied_cells = in.occupied_cells;
    out->bbox_cell_count = in.bbox_cell_count;
    out->missing_baseline_cells = in.missing_baseline_cells;
    out->unfilled_hole_cells = in.unfilled_hole_cells;
    out->footprint_area_m2 = in.footprint_area_m2;
    out->coverage_ratio = in.coverage_ratio;
    out->mean_height_m = in.mean_height_m;
    out->max_height_m = in.max_height_m;
    out->aabb_volume_m3 = in.aabb_volume_m3;
    out->obb_volume_m3 = in.obb_volume_m3;
    out->convex_hull_volume_m3 = in.convex_hull_volume_m3;
    const size_t message_len = in.message.size() < VM_MESSAGE_MAX - 1 ? in.message.size() : VM_MESSAGE_MAX - 1;
    for (size_t i = 0; i < message_len; ++i) {
        out->message[i] = in.message[i];
    }
    out->message[message_len] = '\0';
}

} // namespace



struct vm_owned_cloud {
    vm::PointCloud cloud;
};

struct vm_baseline {
    vm::BaselineModel model;
};

struct vm_components {
    vm::FoodComponents components;
};

struct vm_pipeline {
    vm::VolumePipeline pipeline;
};



extern "C" double vm_length_unit_to_meter_scale(vm_length_unit_t unit) {
    try {
        return vm::length_unit_to_meter_scale(static_cast<vm::LengthUnit>(unit));
    } catch (...) {
        return 1.0;
    }
}



extern "C" const char* vm_status_to_string(vm_status_t status) { return vm::status_to_string(from_c(status)); }



extern "C" int vm_is_finite(const vm_point3f_t* point) {
    if (point == nullptr) {
        return 0;
    }
    return vm::is_finite(vm::Point3f{point->x, point->y, point->z}) ? 1 : 0;
}



extern "C" void vm_config_init(vm_config_t* cfg) {
    if (cfg == nullptr) {
        return;
    }
    const vm::MeasurementConfig defaults{};
    cfg->input_unit = static_cast<vm_length_unit_t>(defaults.input_unit);
    cfg->use_roi = defaults.use_roi ? 1 : 0;
    cfg->roi.min_x = defaults.roi.min_x;
    cfg->roi.max_x = defaults.roi.max_x;
    cfg->roi.min_y = defaults.roi.min_y;
    cfg->roi.max_y = defaults.roi.max_y;
    cfg->roi.min_z = defaults.roi.min_z;
    cfg->roi.max_z = defaults.roi.max_z;
    cfg->voxel_size_m = defaults.voxel_size_m;
    cfg->plane_distance_threshold_m = defaults.plane_distance_threshold_m;
    cfg->plane_ransac_iterations = defaults.plane_ransac_iterations;
    cfg->baseline_max_surface_height_m = defaults.baseline_max_surface_height_m;
    cfg->remove_secondary_plane = defaults.remove_secondary_plane ? 1 : 0;
    cfg->secondary_plane_distance_threshold_m = defaults.secondary_plane_distance_threshold_m;
    cfg->cluster_eps_m = defaults.cluster_eps_m;
    cfg->foreground_cluster_eps_m = defaults.foreground_cluster_eps_m;
    cfg->cluster_min_points = defaults.cluster_min_points;
    cfg->selection_mode = static_cast<vm_selection_mode_t>(defaults.selection_mode);
    cfg->selected_labels = nullptr;
    cfg->selected_label_count = 0;
    cfg->integration_resolution_m = defaults.integration_resolution_m;
    cfg->roi_border_margin_m = defaults.roi_border_margin_m;
    cfg->min_height_m = defaults.min_height_m;
    cfg->max_height_m = defaults.max_height_m;
    cfg->baseline_fill_radius_cells = defaults.baseline_fill_radius_cells;
    cfg->hole_fill_max_cells = defaults.hole_fill_max_cells;
    cfg->hole_fill_neighbor_radius_cells = defaults.hole_fill_neighbor_radius_cells;
    cfg->hole_fill_max_neighbor_height_delta_m = defaults.hole_fill_max_neighbor_height_delta_m;
    cfg->curve_fill_max_hole_area_cm2 = defaults.curve_fill_max_hole_area_cm2;
    cfg->curve_fill_max_component_area_ratio = defaults.curve_fill_max_component_area_ratio;
    cfg->curve_fill_max_imputed_ratio = defaults.curve_fill_max_imputed_ratio;
    cfg->curve_fill_rim_radius_cells = defaults.curve_fill_rim_radius_cells;
    cfg->curve_fill_min_rim_samples = defaults.curve_fill_min_rim_samples;
    cfg->curve_fill_min_rim_coverage = defaults.curve_fill_min_rim_coverage;
    cfg->curve_fill_max_fit_rmse_m = defaults.curve_fill_max_fit_rmse_m;
    cfg->curve_fill_max_prediction_rise_m = defaults.curve_fill_max_prediction_rise_m;
}



extern "C" vm_owned_cloud_t* vm_load_pcd(const char* path) {
    try {
        vm::PointCloud cloud = vm::load_pcd(path == nullptr ? "" : path);
        if (cloud.points.empty()) {
            return nullptr;
        }
        vm_owned_cloud_t* out = new vm_owned_cloud;
        out->cloud = std::move(cloud);
        return out;
    } catch (...) {
        return nullptr;
    }
}



extern "C" vm_cloud_t vm_owned_cloud_view(const vm_owned_cloud_t* cloud) {
    vm_cloud_t view{nullptr, 0};
    if (cloud == nullptr) {
        return view;
    }
    view.points = reinterpret_cast<const vm_point3f_t*>(cloud->cloud.points.data());
    view.count = cloud->cloud.points.size();
    return view;
}



extern "C" void vm_owned_cloud_destroy(vm_owned_cloud_t* cloud) { delete cloud; }



extern "C" vm_status_t vm_preprocess_cloud(const vm_cloud_t* input, const vm_config_t* cfg,
                                           vm_owned_cloud_t** out_cloud, size_t* out_input_points,
                                           size_t* out_retained_points) {
    try {
        if (out_cloud != nullptr) {
            *out_cloud = nullptr;
        }
        vm::PreprocessResult pre{};
        const vm::MeasurementStatus status = vm::preprocess_cloud(to_cpp_cloud(input), to_cpp_config(cfg), pre);
        if (status != vm::MeasurementStatus::kSuccess) {
            return to_c(status);
        }
        if (out_cloud != nullptr) {
            vm_owned_cloud_t* out = new vm_owned_cloud;
            out->cloud = std::move(pre.cloud);
            *out_cloud = out;
        }
        if (out_input_points != nullptr) {
            *out_input_points = pre.input_points;
        }
        if (out_retained_points != nullptr) {
            *out_retained_points = pre.retained_points;
        }
        return VM_STATUS_SUCCESS;
    } catch (...) {
        if (out_cloud != nullptr) {
            *out_cloud = nullptr;
        }
        return VM_STATUS_INVALID_CONFIG;
    }
}



extern "C" vm_owned_cloud_t* vm_voxel_downsample(const vm_cloud_t* cloud, double voxel_size) {
    try {
        vm::PointCloud downsampled = vm::voxel_downsample(to_cpp_cloud(cloud), voxel_size);
        if (downsampled.points.empty()) {
            return nullptr;
        }
        vm_owned_cloud_t* out = new vm_owned_cloud;
        out->cloud = std::move(downsampled);
        return out;
    } catch (...) {
        return nullptr;
    }
}



extern "C" vm_status_t vm_dbscan_labels(const vm_cloud_t* cloud, double eps, int min_points, int* labels,
                                        size_t* out_label_count) {
    try {
        if (cloud == nullptr || labels == nullptr) {
            return VM_STATUS_INVALID_CONFIG;
        }
        const std::vector<int> result = vm::dbscan_labels(to_cpp_points(cloud), eps, min_points);
        for (size_t i = 0; i < result.size(); ++i) {
            labels[i] = result[i];
        }
        if (out_label_count != nullptr) {
            *out_label_count = result.size();
        }
        return VM_STATUS_SUCCESS;
    } catch (...) {
        return VM_STATUS_INVALID_CONFIG;
    }
}



extern "C" vm_status_t vm_fit_plane_ransac(const vm_cloud_t* cloud, double distance_threshold_m, int iterations,
                                           vm_plane_t* out_plane, size_t* inlier_indices, size_t* out_inlier_count) {
    try {
        if (cloud == nullptr || out_plane == nullptr) {
            return VM_STATUS_INVALID_CONFIG;
        }
        vm::Plane plane;
        std::vector<size_t> inliers;
        const vm::MeasurementStatus status =
            vm::fit_plane_ransac(to_cpp_cloud(cloud), distance_threshold_m, iterations, plane, inliers);
        if (status != vm::MeasurementStatus::kSuccess) {
            return to_c(status);
        }
        out_plane->nx = plane.nx;
        out_plane->ny = plane.ny;
        out_plane->nz = plane.nz;
        out_plane->d = plane.d;
        if (inlier_indices != nullptr) {
            for (size_t i = 0; i < inliers.size(); ++i) {
                inlier_indices[i] = inliers[i];
            }
        }
        if (out_inlier_count != nullptr) {
            *out_inlier_count = inliers.size();
        }
        return VM_STATUS_SUCCESS;
    } catch (...) {
        return VM_STATUS_INVALID_CONFIG;
    }
}



extern "C" vm_status_t vm_remove_dominant_plane(const vm_cloud_t* cloud, const vm_config_t* cfg,
                                                vm_owned_cloud_t** out_remaining) {
    try {
        if (out_remaining != nullptr) {
            *out_remaining = nullptr;
        }
        vm::PointCloud remaining;
        const vm::MeasurementStatus status =
            vm::remove_dominant_plane(to_cpp_cloud(cloud), to_cpp_config(cfg), remaining);
        if (status != vm::MeasurementStatus::kSuccess) {
            return to_c(status);
        }
        if (out_remaining != nullptr) {
            vm_owned_cloud_t* out = new vm_owned_cloud;
            out->cloud = std::move(remaining);
            *out_remaining = out;
        }
        return VM_STATUS_SUCCESS;
    } catch (...) {
        if (out_remaining != nullptr) {
            *out_remaining = nullptr;
        }
        return VM_STATUS_INVALID_CONFIG;
    }
}



extern "C" vm_status_t vm_build_baseline_model(const vm_cloud_t* baseline_frames, size_t frame_count,
                                               const vm_cloud_t* orientation_points, const vm_config_t* cfg,
                                               vm_baseline_t** out_baseline) {
    try {
        if (out_baseline != nullptr) {
            *out_baseline = nullptr;
        }
        vm::BaselineModel model;
        const vm::PointCloud orientation =
            orientation_points == nullptr ? vm::PointCloud{} : to_cpp_cloud(orientation_points);
        const vm::MeasurementStatus status = vm::build_baseline_model(to_cpp_clouds(baseline_frames, frame_count),
                                                                      orientation, to_cpp_config(cfg), model);
        if (status != vm::MeasurementStatus::kSuccess) {
            return to_c(status);
        }
        if (out_baseline != nullptr) {
            vm_baseline_t* out = new vm_baseline;
            out->model = std::move(model);
            *out_baseline = out;
        }
        return VM_STATUS_SUCCESS;
    } catch (...) {
        if (out_baseline != nullptr) {
            *out_baseline = nullptr;
        }
        return VM_STATUS_INVALID_CONFIG;
    }
}



extern "C" size_t vm_baseline_frame_count(const vm_baseline_t* baseline) {
    return baseline == nullptr ? 0 : baseline->model.frame_count;
}



extern "C" size_t vm_baseline_cell_count(const vm_baseline_t* baseline) {
    return baseline == nullptr ? 0 : baseline->model.cell_count;
}



extern "C" void vm_baseline_destroy(vm_baseline_t* baseline) { delete baseline; }



extern "C" vm_status_t vm_extract_food_components(const vm_cloud_t* food, const vm_baseline_t* baseline,
                                                  const vm_config_t* cfg, vm_components_t** out_components) {
    try {
        if (out_components != nullptr) {
            *out_components = nullptr;
        }
        if (food == nullptr || baseline == nullptr) {
            return VM_STATUS_INVALID_CONFIG;
        }
        vm::FoodComponents components;
        const vm::MeasurementStatus status =
            vm::extract_food_components(to_cpp_cloud(food), baseline->model, to_cpp_config(cfg), components);
        if (status != vm::MeasurementStatus::kSuccess) {
            return to_c(status);
        }
        if (out_components != nullptr) {
            vm_components_t* out = new vm_components;
            out->components = std::move(components);
            *out_components = out;
        }
        return VM_STATUS_SUCCESS;
    } catch (...) {
        if (out_components != nullptr) {
            *out_components = nullptr;
        }
        return VM_STATUS_INVALID_CONFIG;
    }
}



extern "C" size_t vm_components_cluster_count(const vm_components_t* components) {
    return components == nullptr ? 0 : components->components.cluster_count;
}



extern "C" size_t vm_components_count(const vm_components_t* components) {
    return components == nullptr ? 0 : components->components.labels.size();
}



extern "C" void vm_components_destroy(vm_components_t* components) { delete components; }



extern "C" vm_status_t vm_measure_component_volume(const vm_components_t* components, const vm_baseline_t* baseline,
                                                   const vm_config_t* cfg, vm_component_volume_t* out_volume) {
    try {
        if (components == nullptr || baseline == nullptr || out_volume == nullptr) {
            return VM_STATUS_INVALID_CONFIG;
        }
        vm::ComponentVolumeEstimate volume;
        const vm::MeasurementStatus status =
            vm::measure_component_volume(components->components, baseline->model, to_cpp_config(cfg), volume);
        if (status != vm::MeasurementStatus::kSuccess) {
            return to_c(status);
        }
        fill_component_volume(volume, out_volume);
        return VM_STATUS_SUCCESS;
    } catch (...) {
        return VM_STATUS_INVALID_CONFIG;
    }
}



extern "C" vm_pipeline_t* vm_pipeline_create(void) {
    try {
        return new vm_pipeline;
    } catch (...) {
        return nullptr;
    }
}



extern "C" void vm_pipeline_destroy(vm_pipeline_t* pipeline) { delete pipeline; }



extern "C" vm_status_t vm_measure(vm_pipeline_t* pipeline, const vm_cloud_t* baseline_frames, size_t baseline_count,
                                  const vm_cloud_t* food, const vm_config_t* cfg, vm_estimate_t* out_estimate) {
    try {
        if (pipeline == nullptr || out_estimate == nullptr) {
            return VM_STATUS_INVALID_CONFIG;
        }
        const vm::PointCloud food_cloud = food == nullptr ? vm::PointCloud{} : to_cpp_cloud(food);
        const vm::VolumeEstimate estimate =
            pipeline->pipeline.measure(to_cpp_clouds(baseline_frames, baseline_count), food_cloud, to_cpp_config(cfg));
        fill_estimate(estimate, out_estimate);
        return out_estimate->status;
    } catch (...) {
        if (out_estimate != nullptr) {
            out_estimate->status = VM_STATUS_INVALID_CONFIG;
        }
        return VM_STATUS_INVALID_CONFIG;
    }
}



extern "C" void vm_log_set_level(vm_log_level_t level) { vm::log_set_level(static_cast<vm::LogLevel>(level)); }



extern "C" vm_log_level_t vm_log_get_level(void) { return static_cast<vm_log_level_t>(vm::log_get_level()); }



extern "C" void vm_log_set_console(int enabled) { vm::log_set_console(enabled != 0); }



extern "C" void vm_log_set_file(const char* path, vm_log_file_mode_t mode) {
    vm::log_set_file(path == nullptr ? "" : path,
                     mode == VM_LOG_FILE_TRUNCATE ? vm::LogFileMode::kTruncate : vm::LogFileMode::kAppend);
}



extern "C" void vm_log_write(vm_log_level_t level, const char* message) {
    vm::log_write(static_cast<vm::LogLevel>(level), message == nullptr ? "" : message);
}
