// Conan::ImportStart
#pragma once
#include <cstddef>
#include <vector>
#include "volume_types.hpp"
// Conan::ImportEnd



namespace vm {



/**
 * @brief [en] Output of the point-cloud preprocessing stage.
 * @brief [zh] 点云预处理阶段的输出。
 * @exporter
 */
struct PreprocessResult {
    PointCloud cloud;
    std::size_t input_points = 0;
    std::size_t retained_points = 0;
};



/**
 * @brief [en] Validates, unit-normalizes, crops, voxel-downsamples, and de-noises the input cloud.
 * @brief [zh] 校验、单位归一、裁剪、体素降采样并去噪输入点云。
 * @param input [en] Raw input cloud in `cfg.input_unit`.
 * @param input [zh] 以 `cfg.input_unit` 为单位的原始输入点云。
 * @param cfg [en] Measurement configuration.
 * @param cfg [zh] 测量配置。
 * @param out [en] Receives the processed cloud in meters plus point counters.
 * @param out [zh] 接收以米为单位的处理后点云以及点数统计。
 * @return [en] kSuccess, or an input/config error status.
 * @return [zh] kSuccess，或输入/配置错误状态。
 * @exporter
 */
MeasurementStatus preprocess_cloud(const PointCloud& input, const MeasurementConfig& cfg, PreprocessResult& out);



/**
 * @brief [en] Voxel downsampling matching Open3D `voxel_down_sample`: each occupied voxel keeps the centroid of its points.
 * @brief [zh] 与 Open3D `voxel_down_sample` 一致的体素降采样：每个被占用的体素保留其点的质心。
 * @param cloud [en] Input point cloud.
 * @param cloud [zh] 输入点云。
 * @param voxel_size [en] Voxel edge length in the cloud's units.
 * @param voxel_size [zh] 体素边长（点云单位）。
 * @return [en] Downsampled point cloud (one centroid per occupied voxel).
 * @return [zh] 降采样后的点云（每个被占用体素一个质心）。
 * @exporter
 */
PointCloud voxel_downsample(const PointCloud& cloud, double voxel_size);



/**
 * @brief [en] Density-based spatial clustering (DBSCAN) matching Open3D `cluster_dbscan`.
 * @brief [zh] 与 Open3D `cluster_dbscan` 语义一致的密度聚类（DBSCAN）。
 * @param points [en] Input points.
 * @param points [zh] 输入点。
 * @param eps [en] Neighborhood radius in the point-cloud coordinate units.
 * @param eps [zh] 邻域半径（点云坐标单位）。
 * @param min_points [en] Minimum neighborhood size, including the point itself, required to form a cluster.
 * @param min_points [zh] 构成簇所需的最小邻域点数（含自身）。
 * @return [en] Per-point cluster label in input order; -1 marks noise.
 * @return [zh] 按输入顺序返回的逐点簇标签；-1 表示噪声。
 * @exporter
 */
std::vector<int> dbscan_labels(const std::vector<Point3f>& points, double eps, int min_points);



/**
 * @brief [en] Fits a dominant plane from a point cloud with Open3D-equivalent RANSAC.
 * @brief [zh] 用与 Open3D 等价的 RANSAC 从点云拟合主平面。
 * @param cloud_m [en] Points in metres.
 * @param cloud_m [zh] 以米为单位的点。
 * @param distance_threshold_m [en] RANSAC inlier distance threshold.
 * @param distance_threshold_m [zh] RANSAC 内点距离阈值。
 * @param iterations [en] RANSAC iteration count.
 * @param iterations [zh] RANSAC 迭代次数。
 * @param out_plane [en] Receives the normalized Hessian plane.
 * @param out_plane [zh] 接收归一化的 Hessian 平面。
 * @param inlier_indices [en] Receives the inlier point indices into `cloud_m`.
 * @param inlier_indices [zh] 接收指向 `cloud_m` 的内点索引。
 * @return [en] kSuccess or kPlaneNotFound.
 * @return [zh] kSuccess 或 kPlaneNotFound。
 * @exporter
 */
MeasurementStatus fit_plane_ransac(const PointCloud& cloud_m, double distance_threshold_m, int iterations,
                                   Plane& out_plane, std::vector<std::size_t>& inlier_indices);



/**
 * @brief [en] Removes the dominant background plane(s) from a preprocessed food cloud.
 * @brief [zh] 从预处理食物点云中移除主背景平面（及可选的次级平面）。
 * @param cloud_m [en] Preprocessed points in metres.
 * @param cloud_m [zh] 以米为单位的预处理点。
 * @param cfg [en] Measurement configuration.
 * @param cfg [zh] 测量配置。
 * @param remaining [en] Receives the points that are not background-plane inliers.
 * @param remaining [zh] 接收不属于背景平面内点的剩余点。
 * @return [en] kSuccess or kInvalidConfig.
 * @return [zh] kSuccess 或 kInvalidConfig。
 * @exporter
 */
MeasurementStatus remove_dominant_plane(const PointCloud& cloud_m, const MeasurementConfig& cfg, PointCloud& remaining);



} // namespace vm
