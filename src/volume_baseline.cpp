// Conan::ImportStart
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <map>
#include <memory>
#include <utility>
#include <vector>
#include "volume_baseline.hpp"
#include "volume_internal.hpp"
#include "volume_pointcloudprocess.hpp"
// Conan::ImportEnd



namespace vm {



namespace {

constexpr double kBaselineMinHeightM = -0.003;
constexpr double kTangentEps = 1.0e-8;

double dot3(double ax, double ay, double az, double bx, double by, double bz) { return ax * bx + ay * by + az * bz; }

PlaneFrame make_frame(double nx, double ny, double nz, double ox, double oy, double oz) {
    PlaneFrame f{};
    f.ox = ox;
    f.oy = oy;
    f.oz = oz;
    f.nx = nx;
    f.ny = ny;
    f.nz = nz;

    static const double refs[3][3] = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
    for (const auto& ref : refs) {
        const double t = dot3(ref[0], ref[1], ref[2], nx, ny, nz);
        double tx = ref[0] - t * nx;
        double ty = ref[1] - t * ny;
        double tz = ref[2] - t * nz;
        const double tn = std::sqrt(tx * tx + ty * ty + tz * tz);
        if (tn > kTangentEps) {
            const double ux = tx / tn;
            const double uy = ty / tn;
            const double uz = tz / tn;
            double vx = ny * uz - nz * uy;
            double vy = nz * ux - nx * uz;
            double vz = nx * uy - ny * ux;
            const double vn = std::sqrt(vx * vx + vy * vy + vz * vz);
            if (vn > kTangentEps) {
                vx /= vn;
                vy /= vn;
                vz /= vn;
            }
            f.ux = ux;
            f.uy = uy;
            f.uz = uz;
            f.vx = vx;
            f.vy = vy;
            f.vz = vz;
            return f;
        }
    }
    return f;
}

bool build_plane_roi(const BaselineData& baseline, double border_margin_m, PlaneRoi& out) {
    out = PlaneRoi{};
    if (border_margin_m < 0.0 || baseline.height_by_cell.empty()) {
        return false;
    }

    const double u_min = baseline.bbox_u_min_m + border_margin_m;
    const double u_max = baseline.bbox_u_max_m - border_margin_m;
    const double v_min = baseline.bbox_v_min_m + border_margin_m;
    const double v_max = baseline.bbox_v_max_m - border_margin_m;
    if (u_min >= u_max || v_min >= v_max) {
        return false;
    }

    out.u_min_m = u_min;
    out.u_max_m = u_max;
    out.v_min_m = v_min;
    out.v_max_m = v_max;
    out.border_margin_m = border_margin_m;
    return true;
}

} // namespace



#ifndef __ARM_EABI__

namespace {

PointCloud scale_to_meters(const PointCloud& cloud, const MeasurementConfig& cfg) {
    const double scale = length_unit_to_meter_scale(cfg.input_unit);
    PointCloud out;
    out.points.reserve(cloud.points.size());
    for (const auto& p : cloud.points) {
        // Invalid depth returns are dropped per point instead of aborting the whole frame.
        if (!is_finite(p)) {
            continue;
        }
        out.points.push_back(Point3f{static_cast<float>(static_cast<double>(p.x) * scale),
                                     static_cast<float>(static_cast<double>(p.y) * scale),
                                     static_cast<float>(static_cast<double>(p.z) * scale)});
    }
    return out;
}

} // namespace

#endif // __ARM_EABI__



/**
 * @brief [en] Builds the empty-oven baseline model from one or more baseline frames.
 * @brief [zh] 从一帧或多帧空炉点云建立空炉基线模型。
 * @attacher
 */
MeasurementStatus build_baseline_model(const std::vector<PointCloud>& baseline_frames,
                                       const PointCloud& orientation_points, const MeasurementConfig& cfg,
                                       BaselineModel& out) {
#ifdef __ARM_EABI__
    (void)baseline_frames;
    (void)orientation_points;
    (void)cfg;
    (void)out;
    return MeasurementStatus::kUnsupportedPlatform;
#else
    out = BaselineModel{};
    auto data = std::make_shared<BaselineData>();

    if (baseline_frames.empty()) {
        return MeasurementStatus::kEmptyBaseline;
    }
    if (!(cfg.integration_resolution_m > 0.0F) || !(cfg.voxel_size_m > 0.0F) ||
        !(cfg.plane_distance_threshold_m > 0.0F) || cfg.plane_ransac_iterations <= 0 ||
        !(cfg.baseline_max_surface_height_m > 0.0F)) {
        return MeasurementStatus::kInvalidConfig;
    }
    for (const auto& frame : baseline_frames) {
        if (frame.points.empty()) {
            return MeasurementStatus::kEmptyBaseline;
        }
        // A frame is rejected only when it contributes nothing usable at all.
        if (std::none_of(frame.points.begin(), frame.points.end(), [](const Point3f& p) { return is_finite(p); })) {
            return MeasurementStatus::kNonFiniteBaseline;
        }
    }

    // The reference plane comes from the first frame after voxel downsampling.
    const PointCloud first_m = voxel_downsample(scale_to_meters(baseline_frames[0], cfg), cfg.voxel_size_m);

    Plane plane{};
    std::vector<std::size_t> inliers;
    MeasurementStatus st =
        fit_plane_ransac(first_m, cfg.plane_distance_threshold_m, cfg.plane_ransac_iterations, plane, inliers);
    if (st != MeasurementStatus::kSuccess) {
        return st;
    }

    // Origin is the mean of the plane inliers.
    double ox = 0.0;
    double oy = 0.0;
    double oz = 0.0;
    for (const std::size_t idx : inliers) {
        ox += first_m.points[idx].x;
        oy += first_m.points[idx].y;
        oz += first_m.points[idx].z;
    }
    const double inv_n = 1.0 / static_cast<double>(inliers.size());
    ox *= inv_n;
    oy *= inv_n;
    oz *= inv_n;

    double nx = plane.nx;
    double ny = plane.ny;
    double nz = plane.nz;

    // Orient the normal toward the food side when orientation points are available.
    if (!orientation_points.points.empty()) {
        std::vector<double> heights;
        heights.reserve(orientation_points.points.size());
        for (const auto& p : orientation_points.points) {
            if (!is_finite(p)) {
                continue;
            }
            heights.push_back((static_cast<double>(p.x) - ox) * nx + (static_cast<double>(p.y) - oy) * ny +
                              (static_cast<double>(p.z) - oz) * nz);
        }
        if (!heights.empty() && median(std::move(heights)) < 0.0) {
            nx = -nx;
            ny = -ny;
            nz = -nz;
        }
    }

    data->plane.nx = static_cast<float>(nx);
    data->plane.ny = static_cast<float>(ny);
    data->plane.nz = static_cast<float>(nz);
    data->plane.d = static_cast<float>(nx * ox + ny * oy + nz * oz);
    data->frame = make_frame(nx, ny, nz, ox, oy, oz);
    data->cell_size_m = cfg.integration_resolution_m;

    std::map<CellKey, std::vector<double>> samples_by_cell;
    for (const auto& frame : baseline_frames) {
        const PointCloud frame_m = scale_to_meters(frame, cfg);
        for (const auto& p : frame_m.points) {
            double u = 0.0;
            double v = 0.0;
            double h = 0.0;
            project_to_plane_frame(data->frame, p, u, v, h);
            if (!std::isfinite(h) || h < kBaselineMinHeightM || h > cfg.baseline_max_surface_height_m) {
                continue;
            }
            samples_by_cell[cell_index_of(u, v, data->cell_size_m)].push_back(h);
        }
    }

    if (samples_by_cell.empty()) {
        return MeasurementStatus::kBaselineNoCells;
    }

    bool first_cell = true;
    for (const auto& entry : samples_by_cell) {
        const CellKey key = entry.first;
        const double center_u = (static_cast<double>(key.first) + 0.5) * data->cell_size_m;
        const double center_v = (static_cast<double>(key.second) + 0.5) * data->cell_size_m;
        data->height_by_cell[key] = median(entry.second);
        if (first_cell) {
            data->bbox_u_min_m = center_u;
            data->bbox_u_max_m = center_u;
            data->bbox_v_min_m = center_v;
            data->bbox_v_max_m = center_v;
            first_cell = false;
        } else {
            data->bbox_u_min_m = std::min(data->bbox_u_min_m, center_u);
            data->bbox_u_max_m = std::max(data->bbox_u_max_m, center_u);
            data->bbox_v_min_m = std::min(data->bbox_v_min_m, center_v);
            data->bbox_v_max_m = std::max(data->bbox_v_max_m, center_v);
        }
    }

    PlaneRoi roi{};
    if (!build_plane_roi(*data, cfg.roi_border_margin_m, roi)) {
        return MeasurementStatus::kInvalidConfig;
    }
    data->roi = roi;

    out.frame_count = baseline_frames.size();
    out.cell_count = data->height_by_cell.size();
    out.data = std::move(data);
    return MeasurementStatus::kSuccess;
#endif // __ARM_EABI__
}



} // namespace vm