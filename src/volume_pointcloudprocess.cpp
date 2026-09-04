// Conan::ImportStart
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <map>
#include <memory>
#include <random>
#include <tuple>
#include <utility>
#include <vector>
#include "volume_pointcloudprocess.hpp"
#ifndef __ARM_EABI__
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#endif
// Conan::ImportEnd



namespace vm {



namespace {

double signed_height(const Plane& plane, const Point3f& p) {
    return static_cast<double>(plane.nx) * p.x + static_cast<double>(plane.ny) * p.y +
           static_cast<double>(plane.nz) * p.z - static_cast<double>(plane.d);
}

} // namespace



#ifndef __ARM_EABI__

namespace {

// Open3D's RANSAC bookkeeping: a larger inlier ratio wins, ties are broken by the smaller RMSE.
struct RansacResult {
    double fitness = 0.0;
    double inlier_rmse = 0.0;
};

bool is_better_ransac(const RansacResult& candidate, const RansacResult& best) {
    return candidate.fitness > best.fitness ||
           (candidate.fitness == best.fitness && candidate.inlier_rmse < best.inlier_rmse);
}

// Open3D's default `probability` for PointCloud::SegmentPlane, and the fixed 3-point sample size.
constexpr double kRansacProbability = 0.99999999;
constexpr int kRansacSampleSize = 3;
constexpr double kDegenerateNormalNorm = 1.0e-12;
// Matches `o3d.utility.random.seed(43)` in the PCD-IM reference script.
constexpr unsigned int kRansacSeed = 43U;

} // namespace

#endif // __ARM_EABI__



PointCloud voxel_downsample(const PointCloud& cloud, double voxel_size) {
    PointCloud out;
    if (cloud.points.empty() || !(voxel_size > 0.0)) {
        return out;
    }

    double min_x = static_cast<double>(cloud.points[0].x);
    double min_y = static_cast<double>(cloud.points[0].y);
    double min_z = static_cast<double>(cloud.points[0].z);
    for (const auto& p : cloud.points) {
        min_x = std::min(min_x, static_cast<double>(p.x));
        min_y = std::min(min_y, static_cast<double>(p.y));
        min_z = std::min(min_z, static_cast<double>(p.z));
    }
    const double ref_x = min_x - 0.5 * voxel_size;
    const double ref_y = min_y - 0.5 * voxel_size;
    const double ref_z = min_z - 0.5 * voxel_size;

    struct Accumulator {
        double sum_x = 0.0;
        double sum_y = 0.0;
        double sum_z = 0.0;
        std::size_t count = 0;
    };

    std::map<std::tuple<int, int, int>, Accumulator> voxels;
    for (const auto& p : cloud.points) {
        const int ix = static_cast<int>(std::floor((static_cast<double>(p.x) - ref_x) / voxel_size));
        const int iy = static_cast<int>(std::floor((static_cast<double>(p.y) - ref_y) / voxel_size));
        const int iz = static_cast<int>(std::floor((static_cast<double>(p.z) - ref_z) / voxel_size));
        Accumulator& acc = voxels[{ix, iy, iz}];
        acc.sum_x += static_cast<double>(p.x);
        acc.sum_y += static_cast<double>(p.y);
        acc.sum_z += static_cast<double>(p.z);
        acc.count += 1;
    }

    out.points.reserve(voxels.size());
    for (const auto& entry : voxels) {
        const Accumulator& acc = entry.second;
        const double inv = 1.0 / static_cast<double>(acc.count);
        out.points.push_back(Point3f{static_cast<float>(acc.sum_x * inv), static_cast<float>(acc.sum_y * inv),
                                     static_cast<float>(acc.sum_z * inv)});
    }
    return out;
}



/**
 * @brief [en] Validates, unit-normalizes, optionally crops, and voxel-downsamples the input cloud.
 * @brief [zh] 校验、单位归一、可选裁剪并体素降采样输入点云。
 * @attacher
 */
MeasurementStatus preprocess_cloud(const PointCloud& input, const MeasurementConfig& cfg, PreprocessResult& out) {
#ifdef __ARM_EABI__
    (void)input;
    (void)cfg;
    (void)out;
    return MeasurementStatus::kUnsupportedPlatform;
#else
    out = PreprocessResult{};

    if (input.points.empty()) {
        return MeasurementStatus::kEmptyInput;
    }
    // Depth sensors encode invalid returns as NaN/Inf. Those points are dropped one by one, as the
    // reference implementation masks them, instead of aborting the whole measurement. Rejecting is
    // reserved for input that has no usable point at all.
    if (std::none_of(input.points.begin(), input.points.end(), [](const Point3f& p) { return is_finite(p); })) {
        return MeasurementStatus::kNonFiniteInput;
    }

    const double scale = length_unit_to_meter_scale(cfg.input_unit);
    if (!(scale > 0.0) || !(cfg.voxel_size_m > 0.0F)) {
        return MeasurementStatus::kInvalidConfig;
    }
    if (cfg.use_roi &&
        !(cfg.roi.min_x < cfg.roi.max_x && cfg.roi.min_y < cfg.roi.max_y && cfg.roi.min_z < cfg.roi.max_z)) {
        return MeasurementStatus::kInvalidConfig;
    }

    // Unit-normalize to metres, dropping non-finite points on the way.
    PointCloud scaled;
    scaled.points.reserve(input.points.size());
    for (const auto& p : input.points) {
        if (!is_finite(p)) {
            continue;
        }
        scaled.points.push_back(Point3f{static_cast<float>(static_cast<double>(p.x) * scale),
                                        static_cast<float>(static_cast<double>(p.y) * scale),
                                        static_cast<float>(static_cast<double>(p.z) * scale)});
    }

    // Optional axis-aligned ROI crop.
    PointCloud cropped = scaled;
    if (cfg.use_roi) {
        PointCloud filtered;
        filtered.points.reserve(cropped.points.size());
        for (const auto& p : cropped.points) {
            if (p.x >= cfg.roi.min_x && p.x <= cfg.roi.max_x && p.y >= cfg.roi.min_y && p.y <= cfg.roi.max_y &&
                p.z >= cfg.roi.min_z && p.z <= cfg.roi.max_z) {
                filtered.points.push_back(p);
            }
        }
        cropped = std::move(filtered);
    }

    // Voxel downsample (Open3D-equivalent centroid rule).
    const PointCloud voxeled = voxel_downsample(cropped, cfg.voxel_size_m);

    out.cloud = voxeled;
    out.input_points = input.points.size();
    out.retained_points = out.cloud.points.size();

    if (out.cloud.points.empty()) {
        return MeasurementStatus::kEmptyInput;
    }
    return MeasurementStatus::kSuccess;
#endif // __ARM_EABI__
}



/**
 * @brief [en] Density-based spatial clustering (DBSCAN) matching Open3D `cluster_dbscan`.
 * @brief [zh] 与 Open3D `cluster_dbscan` 语义一致的密度聚类（DBSCAN）。
 * @attacher
 */
std::vector<int> dbscan_labels(const std::vector<Point3f>& points, double eps, int min_points) {
#ifdef __ARM_EABI__
    (void)points;
    (void)eps;
    (void)min_points;
    return std::vector<int>(points.size(), -1);
#else
    const std::size_t point_count = points.size();
    if (point_count == 0) {
        return {};
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->reserve(point_count);
    for (const auto& p : points) {
        cloud->push_back(pcl::PointXYZ(p.x, p.y, p.z));
    }

    auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
    tree->setInputCloud(cloud);

    // Precompute the eps-neighborhood (including the query point itself) for every point.
    std::vector<std::vector<int>> neighborhoods(point_count);
    for (std::size_t i = 0; i < point_count; ++i) {
        std::vector<int> indices;
        std::vector<float> squared_distances;
        tree->radiusSearch((*cloud)[i], eps, indices, squared_distances);
        neighborhoods[i] = std::move(indices);
    }

    std::vector<int> labels(point_count, -2); // -2 unprocessed, -1 noise
    int cluster_label = 0;
    for (std::size_t i = 0; i < point_count; ++i) {
        if (labels[i] != -2) {
            continue;
        }
        if (neighborhoods[i].size() < static_cast<std::size_t>(min_points)) {
            labels[i] = -1; // not a core point
            continue;
        }

        labels[i] = cluster_label;
        std::vector<int> queue = neighborhoods[i];
        std::size_t head = 0;
        while (head < queue.size()) {
            const int neighbor = queue[head++];
            if (labels[static_cast<std::size_t>(neighbor)] == -1) {
                labels[static_cast<std::size_t>(neighbor)] = cluster_label; // border point
            }
            if (labels[static_cast<std::size_t>(neighbor)] != -2) {
                continue;
            }
            labels[static_cast<std::size_t>(neighbor)] = cluster_label;
            if (neighborhoods[static_cast<std::size_t>(neighbor)].size() >= static_cast<std::size_t>(min_points)) {
                const auto& next = neighborhoods[static_cast<std::size_t>(neighbor)];
                queue.insert(queue.end(), next.begin(), next.end());
            }
        }
        ++cluster_label;
    }
    return labels;
#endif // __ARM_EABI__
}



/**
 * @brief [en] Fits a dominant plane from a point cloud with Open3D-equivalent RANSAC.
 * @brief [zh] 用与 Open3D 等价的 RANSAC 从点云拟合主平面。
 * @attacher
 */
MeasurementStatus fit_plane_ransac(const PointCloud& cloud_m, double distance_threshold_m, int iterations,
                                   Plane& out_plane, std::vector<std::size_t>& inlier_indices) {
#ifdef __ARM_EABI__
    (void)cloud_m;
    (void)distance_threshold_m;
    (void)iterations;
    (void)out_plane;
    (void)inlier_indices;
    return MeasurementStatus::kUnsupportedPlatform;
#else
    out_plane = Plane{};
    inlier_indices.clear();

    if (cloud_m.points.empty() || !(distance_threshold_m > 0.0) || iterations <= 0) {
        return MeasurementStatus::kPlaneNotFound;
    }

    const std::size_t n_points = cloud_m.points.size();

    // A 3-point plane model needs at least 3 distinct points; Open3D rejects smaller inputs as well.
    if (n_points < static_cast<std::size_t>(kRansacSampleSize)) {
        return MeasurementStatus::kPlaneNotFound;
    }

    // Open3D computes in double precision; the input is float32 but promoted losslessly.
    std::vector<Eigen::Vector3d> points;
    points.reserve(n_points);
    for (const auto& p : cloud_m.points) {
        points.emplace_back(static_cast<double>(p.x), static_cast<double>(p.y), static_cast<double>(p.z));
    }

    // The engine is local to this call, so the result depends only on the inputs. A process-wide static
    // engine would leak state and make two identical calls disagree. Note that Open3D shares one engine
    // across calls in a script, so its raw sample stream cannot be replayed bit-for-bit here.
    std::mt19937 engine(kRansacSeed);
    std::uniform_int_distribution<int> point_distribution(0, static_cast<int>(n_points - 1));

    // Samples are drawn up front, like Open3D, so the stream does not depend on the early exit below.
    // Each sample holds `kRansacSampleSize` distinct indices; duplicates are redrawn.
    std::vector<std::array<std::size_t, static_cast<std::size_t>(kRansacSampleSize)>> samples;
    samples.reserve(static_cast<std::size_t>(iterations));
    for (int itr = 0; itr < iterations; ++itr) {
        std::array<std::size_t, static_cast<std::size_t>(kRansacSampleSize)> sample{};
        std::size_t drawn = 0;
        while (drawn < sample.size()) {
            const std::size_t candidate = static_cast<std::size_t>(point_distribution(engine));
            bool duplicate = false;
            for (std::size_t k = 0; k < drawn; ++k) {
                if (sample[k] == candidate) {
                    duplicate = true;
                    break;
                }
            }
            if (!duplicate) {
                sample[drawn] = candidate;
                ++drawn;
            }
        }
        samples.push_back(sample);
    }

    RansacResult best{};
    Eigen::Vector3d best_normal = Eigen::Vector3d::Zero();
    double best_offset = 0.0;
    bool have_best = false;
    int break_iteration = iterations;

    for (int itr = 0; itr < iterations; ++itr) {
        // Iterations past the early-exit bound are skipped, mirroring Open3D's break_iteration.
        if (itr > break_iteration) {
            break;
        }

        const std::array<std::size_t, static_cast<std::size_t>(kRansacSampleSize)>& sample =
            samples[static_cast<std::size_t>(itr)];
        const Eigen::Vector3d& p0 = points[sample[0]];
        const Eigen::Vector3d& p1 = points[sample[1]];
        const Eigen::Vector3d& p2 = points[sample[2]];

        Eigen::Vector3d normal = (p1 - p0).cross(p2 - p0);
        if (normal.norm() < kDegenerateNormalNorm) {
            continue;
        }
        normal.normalize();
        const double offset = -normal.dot(p0);

        // Inliers use a strict inequality, exactly like Open3D's EvaluateRANSACBasedOnDistance.
        std::size_t inlier_count = 0;
        double error = 0.0;
        for (std::size_t idx = 0; idx < n_points; ++idx) {
            const double distance = std::fabs(points[idx].dot(normal) + offset);
            if (distance < distance_threshold_m) {
                error += distance * distance;
                ++inlier_count;
            }
        }

        RansacResult current{};
        if (inlier_count > 0) {
            current.fitness = static_cast<double>(inlier_count) / static_cast<double>(n_points);
            current.inlier_rmse = std::sqrt(error / static_cast<double>(inlier_count));
        }
        if (!have_best || is_better_ransac(current, best)) {
            best = current;
            best_normal = normal;
            best_offset = offset;
            have_best = true;
            // Recompute the early-exit bound from the newly accepted model.
            if (best.fitness >= 1.0) {
                break_iteration = 0;
            } else if (best.fitness > 0.0) {
                const double estimate =
                    std::log(1.0 - kRansacProbability) / std::log(1.0 - std::pow(best.fitness, kRansacSampleSize));
                break_iteration = static_cast<int>(std::min(estimate, static_cast<double>(iterations)));
            }
        }
    }

    if (!have_best || best.fitness <= 0.0) {
        return MeasurementStatus::kPlaneNotFound;
    }

    // Open3D recomputes the inlier set from the winning model before refining, so the reported
    // inliers (and the plane origin derived from them) belong to the final model.
    std::vector<std::size_t> final_inliers;
    for (std::size_t idx = 0; idx < n_points; ++idx) {
        if (std::fabs(points[idx].dot(best_normal) + best_offset) < distance_threshold_m) {
            final_inliers.push_back(idx);
        }
    }
    if (final_inliers.empty()) {
        return MeasurementStatus::kPlaneNotFound;
    }

    // Refine the model on all inliers: the plane normal is the smallest eigenvector of the inlier covariance.
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (const std::size_t idx : final_inliers) {
        mean += points[idx];
    }
    mean /= static_cast<double>(final_inliers.size());

    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (const std::size_t idx : final_inliers) {
        const Eigen::Vector3d diff = points[idx] - mean;
        cov += diff * diff.transpose();
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
    Eigen::Vector3d normal = solver.eigenvectors().col(0);
    normal.normalize();

    out_plane.nx = static_cast<float>(normal.x());
    out_plane.ny = static_cast<float>(normal.y());
    out_plane.nz = static_cast<float>(normal.z());
    out_plane.d = static_cast<float>(normal.dot(mean));

    inlier_indices = std::move(final_inliers);
    return MeasurementStatus::kSuccess;
#endif // __ARM_EABI__
}



/**
 * @brief [en] Removes the dominant background plane(s) from a preprocessed food cloud.
 * @brief [zh] 从预处理食物点云中移除主背景平面（及可选的次级平面）。
 * @attacher
 */
MeasurementStatus remove_dominant_plane(const PointCloud& cloud_m, const MeasurementConfig& cfg,
                                        PointCloud& remaining) {
#ifdef __ARM_EABI__
    (void)cloud_m;
    (void)cfg;
    (void)remaining;
    return MeasurementStatus::kUnsupportedPlatform;
#else
    remaining = PointCloud{};
    if (!(cfg.plane_distance_threshold_m > 0.0F) ||
        (cfg.remove_secondary_plane && !(cfg.secondary_plane_distance_threshold_m > 0.0F))) {
        return MeasurementStatus::kInvalidConfig;
    }

    auto keep = [&](const Plane& plane, double threshold) {
        PointCloud kept;
        kept.points.reserve(remaining.points.size());
        for (const auto& p : remaining.points) {
            // Inliers are `|h| < threshold`, so the complement keeps `|h| >= threshold`.
            if (std::fabs(signed_height(plane, p)) >= threshold) {
                kept.points.push_back(p);
            }
        }
        remaining = kept;
    };

    remaining = cloud_m;

    Plane plane{};
    std::vector<std::size_t> inliers;
    if (fit_plane_ransac(cloud_m, cfg.plane_distance_threshold_m, cfg.plane_ransac_iterations, plane, inliers) ==
        MeasurementStatus::kSuccess) {
        keep(plane, cfg.plane_distance_threshold_m);
    }

    if (cfg.remove_secondary_plane) {
        Plane secondary{};
        std::vector<std::size_t> secondary_inliers;
        if (fit_plane_ransac(remaining, cfg.secondary_plane_distance_threshold_m, cfg.plane_ransac_iterations,
                             secondary, secondary_inliers) == MeasurementStatus::kSuccess) {
            keep(secondary, cfg.secondary_plane_distance_threshold_m);
        }
    }
    return MeasurementStatus::kSuccess;
#endif // __ARM_EABI__
}



} // namespace vm
