// Conan::ImportStart
#ifndef VM_C_H
#define VM_C_H
#include <stddef.h>
// Conan::ImportEnd

#ifdef __cplusplus
extern "C" {
#endif



/**
 * @brief [en] Maximum number of food components whose labels are reported in a volume estimate.
 * @brief [zh] 体积估计结果中报告标签的食材组件数上限。
 */
#define VM_MAX_COMPONENTS 16



/**
 * @brief [en] Fixed capacity of the message buffer in a volume estimate.
 * @brief [zh] 体积估计结果中消息缓冲区的固定容量。
 */
#define VM_MESSAGE_MAX 256



/**
 * @brief [en] Outcome status of a volume measurement.
 * @brief [zh] 体积测量的结果状态。
 * @since 2.0
 */
typedef enum vm_status {
    VM_STATUS_SUCCESS = 0,
    VM_STATUS_EMPTY_INPUT = 1,
    VM_STATUS_NON_FINITE_INPUT = 2,
    VM_STATUS_INVALID_CONFIG = 3,
    VM_STATUS_EMPTY_BASELINE = 4,
    VM_STATUS_NON_FINITE_BASELINE = 5,
    VM_STATUS_PLANE_NOT_FOUND = 6,
    VM_STATUS_PLANE_LOW_QUALITY = 7,
    VM_STATUS_BASELINE_NO_CELLS = 8,
    VM_STATUS_FOOD_NOT_FOUND = 9,
    VM_STATUS_INSUFFICIENT_COVERAGE = 10,
    VM_STATUS_UNSUPPORTED_PLATFORM = 11
} vm_status_t;



/**
 * @brief [en] Linear unit of the input point cloud.
 * @brief [zh] 输入点云的线性单位。
 * @since 2.0
 */
typedef enum vm_length_unit { VM_LENGTH_UNIT_METER = 0, VM_LENGTH_UNIT_MILLIMETER = 1 } vm_length_unit_t;



/**
 * @brief [en] How foreground food components are selected.
 * @brief [zh] 前景食材组件的选择方式。
 * @since 2.0
 */
typedef enum vm_selection_mode { VM_SELECTION_ALL_ELIGIBLE = 0, VM_SELECTION_MANUAL = 1 } vm_selection_mode_t;



/**
 * @brief [en] A single 3D point with single precision.
 * @brief [zh] 单精度三维点。
 * @since 2.0
 */
typedef struct vm_point3f {
    float x;
    float y;
    float z;
} vm_point3f_t;



/**
 * @brief [en] A non-owning view of a point cloud (caller-managed buffer).
 * @brief [zh] 点云的非拥有视图（调用方管理缓冲区）。
 * @since 2.0
 */
typedef struct vm_cloud {
    const vm_point3f_t *points;
    size_t count;
} vm_cloud_t;



/**
 * @brief [en] A plane in Hessian form with unit normal; the food side satisfies dot(n, p) > d.
 * @brief [zh] Hessian 形式平面，法向量为单位向量，食材一侧满足 dot(n,p)>d。
 * @since 2.0
 */
typedef struct vm_plane {
    float nx;
    float ny;
    float nz;
    float d;
} vm_plane_t;



/**
 * @brief [en] Axis-aligned region of interest bounds.
 * @brief [zh] 轴对齐的感兴趣区域边界。
 * @since 2.0
 */
typedef struct vm_roi {
    float min_x;
    float max_x;
    float min_y;
    float max_y;
    float min_z;
    float max_z;
} vm_roi_t;



/**
 * @brief [en] Configuration for the PCD-IM volume pipeline.
 * @brief [zh] PCD-IM 体积流水线的配置。
 * @since 2.0
 */
typedef struct vm_config {
    vm_length_unit_t input_unit;

    int use_roi;
    vm_roi_t roi;

    double voxel_size_m;

    double plane_distance_threshold_m;
    int plane_ransac_iterations;
    double baseline_max_surface_height_m;

    int remove_secondary_plane;
    double secondary_plane_distance_threshold_m;

    double cluster_eps_m;
    double foreground_cluster_eps_m;
    int cluster_min_points;

    vm_selection_mode_t selection_mode;
    const int *selected_labels;
    size_t selected_label_count;

    double integration_resolution_m;
    double roi_border_margin_m;
    double min_height_m;
    double max_height_m;

    int baseline_fill_radius_cells;

    int hole_fill_max_cells;
    int hole_fill_neighbor_radius_cells;
    double hole_fill_max_neighbor_height_delta_m;

    double curve_fill_max_hole_area_cm2;
    double curve_fill_max_component_area_ratio;
    double curve_fill_max_imputed_ratio;
    int curve_fill_rim_radius_cells;
    int curve_fill_min_rim_samples;
    double curve_fill_min_rim_coverage;
    double curve_fill_max_fit_rmse_m;
    double curve_fill_max_prediction_rise_m;
} vm_config_t;



/**
 * @brief [en] Per-component PCD-IM integration result.
 * @brief [zh] 单组件 PCD-IM 积分结果。
 * @since 2.0
 */
typedef struct vm_component_volume {
    double raw_volume_cm3;
    double interpolated_volume_cm3;
    double volume_cm3;

    size_t top_surface_points;
    size_t measured_cells;
    size_t interpolated_cells;
    size_t occupied_cells;
    size_t bbox_cell_count;
    size_t missing_baseline_cells;
    size_t unfilled_hole_cells;

    double footprint_area_m2;
    double coverage_ratio;
    double mean_height_m;
    double max_height_m;
} vm_component_volume_t;



/**
 * @brief [en] Result of a PCD-IM volume measurement.
 * @brief [zh] PCD-IM 体积测量结果。
 * @since 2.0
 */
typedef struct vm_estimate {
    vm_status_t status;

    double volume_cm3;
    double raw_volume_cm3;
    double interpolated_volume_cm3;
    double uncertainty_cm3;

    size_t input_points;
    size_t downsampled_points;
    size_t cluster_count;
    int selected_cluster_labels[VM_MAX_COMPONENTS];
    size_t selected_cluster_count;
    size_t selected_cluster_points;

    size_t baseline_frames;
    size_t baseline_cell_count;
    size_t component_count;

    size_t top_surface_points;
    size_t measured_cells;
    size_t interpolated_cells;
    size_t occupied_cells;
    size_t bbox_cell_count;
    size_t missing_baseline_cells;
    size_t unfilled_hole_cells;

    double footprint_area_m2;
    double coverage_ratio;
    double mean_height_m;
    double max_height_m;

    double aabb_volume_m3;
    double obb_volume_m3;
    double convex_hull_volume_m3;

    char message[VM_MESSAGE_MAX];
} vm_estimate_t;



/**
 * @brief [en] Owns a heap-allocated point cloud returned by cloud-producing functions.
 * @brief [zh] 拥有由产生点云的函数返回的堆分配点云。
 * @since 2.0
 */
typedef struct vm_owned_cloud vm_owned_cloud_t;



/**
 * @brief [en] Owns a reusable empty-oven baseline model.
 * @brief [zh] 拥有可复用的空炉基线模型。
 * @since 2.0
 */
typedef struct vm_baseline vm_baseline_t;



/**
 * @brief [en] Owns a set of selected foreground food components.
 * @brief [zh] 拥有一组选中的前景食材组件。
 * @since 2.0
 */
typedef struct vm_components vm_components_t;



/**
 * @brief [en] Owns an end-to-end PCD-IM volume pipeline.
 * @brief [zh] 拥有端到端的 PCD-IM 体积流水线。
 * @since 2.0
 */
typedef struct vm_pipeline vm_pipeline_t;



/**
 * @brief [en] Converts a length unit to the meters scale factor.
 * @brief [zh] 将长度单位转换为米的比例因子。
 * @param unit [en] The input length unit.
 * @param unit [zh] 输入长度单位。
 * @return [en] The factor that converts a coordinate in `unit` to meters.
 * @return [zh] 把 `unit` 坐标换算成米的因子。
 * @since 2.0
 */
double vm_length_unit_to_meter_scale(vm_length_unit_t unit);



/**
 * @brief [en] Returns a human-readable description of a measurement status.
 * @brief [zh] 返回测量状态的可读描述。
 * @param status [en] The measurement status.
 * @param status [zh] 测量状态。
 * @return [en] A static description string.
 * @return [zh] 静态描述字符串。
 * @since 2.0
 */
const char *vm_status_to_string(vm_status_t status);



/**
 * @brief [en] Checks whether a point has finite coordinates.
 * @brief [zh] 检查点坐标是否有限。
 * @param point [en] The point to test; must not be NULL.
 * @param point [zh] 待检查的点；不能为 NULL。
 * @return [en] Non-zero when all coordinates are finite.
 * @return [zh] 所有坐标都有限时返回非零。
 * @since 2.0
 */
int vm_is_finite(const vm_point3f_t *point);



/**
 * @brief [en] Fills a configuration with the library defaults.
 * @brief [zh] 用库默认值填充配置。
 * @param cfg [en] Receives the default configuration; must not be NULL.
 * @param cfg [zh] 接收默认配置；不能为 NULL。
 * @since 2.0
 */
void vm_config_init(vm_config_t *cfg);



/**
 * @brief [en] Loads a PCD point-cloud file.
 * @brief [zh] 载入 PCD 点云文件。
 * @param path [en] Path to the PCD file.
 * @param path [zh] PCD 文件路径。
 * @return [en] An owned cloud, or NULL when the file cannot be read or is empty.
 * @return [zh] 拥有型点云；文件无法读取或为空时返回 NULL。
 * @since 2.0
 */
vm_owned_cloud_t *vm_load_pcd(const char *path);



/**
 * @brief [en] Returns a non-owning view of an owned cloud.
 * @brief [zh] 返回拥有型点云的非拥有视图。
 * @param cloud [en] The owned cloud; must not be NULL.
 * @param cloud [zh] 拥有型点云；不能为 NULL。
 * @return [en] A view whose buffer is valid until the cloud is destroyed.
 * @return [zh] 一个视图，其缓冲区在点云销毁前有效。
 * @since 2.0
 */
vm_cloud_t vm_owned_cloud_view(const vm_owned_cloud_t *cloud);



/**
 * @brief [en] Releases an owned cloud.
 * @brief [zh] 释放拥有型点云。
 * @param cloud [en] The cloud to release; may be NULL.
 * @param cloud [zh] 待释放的点云；可为 NULL。
 * @since 2.0
 */
void vm_owned_cloud_destroy(vm_owned_cloud_t *cloud);



/**
 * @brief [en] Validates, unit-normalizes, optionally crops, and voxel-downsamples a cloud.
 * @brief [zh] 校验、单位归一、可选裁剪并体素降采样点云。
 * @param input [en] The raw input cloud.
 * @param input [zh] 原始输入点云。
 * @param cfg [en] Measurement configuration.
 * @param cfg [zh] 测量配置。
 * @param out_cloud [en] Receives the processed cloud, or NULL on failure.
 * @param out_cloud [zh] 接收处理后点云；失败时为 NULL。
 * @param out_input_points [en] Receives the raw input point count; may be NULL.
 * @param out_input_points [zh] 接收原始输入点数；可为 NULL。
 * @param out_retained_points [en] Receives the downsampled point count; may be NULL.
 * @param out_retained_points [zh] 接收降采样后点数；可为 NULL。
 * @return [en] kSuccess, or an input/config error status.
 * @return [zh] kSuccess 或输入/配置错误状态。
 * @since 2.0
 */
vm_status_t vm_preprocess_cloud(const vm_cloud_t *input, const vm_config_t *cfg, vm_owned_cloud_t **out_cloud,
                                size_t *out_input_points, size_t *out_retained_points);



/**
 * @brief [en] Voxel-downsamples a cloud: each occupied voxel keeps the centroid of its points.
 * @brief [zh] 体素降采样：每个被占用体素保留其点的质心。
 * @param cloud [en] Input cloud.
 * @param cloud [zh] 输入点云。
 * @param voxel_size [en] Voxel edge length in the cloud's units.
 * @param voxel_size [zh] 体素边长（点云单位）。
 * @return [en] An owned downsampled cloud, or NULL when the input is empty.
 * @return [zh] 拥有型降采样点云；输入为空时返回 NULL。
 * @since 2.0
 */
vm_owned_cloud_t *vm_voxel_downsample(const vm_cloud_t *cloud, double voxel_size);



/**
 * @brief [en] Density-based clustering (DBSCAN).
 * @brief [zh] 密度聚类（DBSCAN）。
 * @param cloud [en] Input points.
 * @param cloud [zh] 输入点。
 * @param eps [en] Neighborhood radius in the point-cloud coordinate units.
 * @param eps [zh] 邻域半径（点云坐标单位）。
 * @param min_points [en] Minimum neighborhood size, including the point itself.
 * @param min_points [zh] 构成簇所需的最小邻域点数（含自身）。
 * @param labels [en] Receives one label per input point in input order; -1 marks noise. Capacity must be >= cloud->count.
 * @param labels [zh] 按输入顺序接收逐点标签；-1 表示噪声。容量须 >= cloud->count。
 * @param out_label_count [en] Receives the number of labels written; may be NULL.
 * @param out_label_count [zh] 接收写入的标签数；可为 NULL。
 * @return [en] kSuccess or kInvalidConfig.
 * @return [zh] kSuccess 或 kInvalidConfig。
 * @since 2.0
 */
vm_status_t vm_dbscan_labels(const vm_cloud_t *cloud, double eps, int min_points, int *labels, size_t *out_label_count);



/**
 * @brief [en] Fits a dominant plane with RANSAC.
 * @brief [zh] 用 RANSAC 拟合主平面。
 * @param cloud [en] Input points in metres.
 * @param cloud [zh] 以米为单位的输入点。
 * @param distance_threshold_m [en] RANSAC inlier distance threshold.
 * @param distance_threshold_m [zh] RANSAC 内点距离阈值。
 * @param iterations [en] RANSAC iteration count.
 * @param iterations [zh] RANSAC 迭代次数。
 * @param out_plane [en] Receives the normalized Hessian plane.
 * @param out_plane [zh] 接收归一化的 Hessian 平面。
 * @param inlier_indices [en] Receives inlier indices into `cloud`; may be NULL. Capacity must be >= cloud->count.
 * @param inlier_indices [zh] 接收指向 `cloud` 的内点索引；可为 NULL。容量须 >= cloud->count。
 * @param out_inlier_count [en] Receives the number of inliers; may be NULL.
 * @param out_inlier_count [zh] 接收内点数；可为 NULL。
 * @return [en] kSuccess or kPlaneNotFound.
 * @return [zh] kSuccess 或 kPlaneNotFound。
 * @since 2.0
 */
vm_status_t vm_fit_plane_ransac(const vm_cloud_t *cloud, double distance_threshold_m, int iterations,
                                vm_plane_t *out_plane, size_t *inlier_indices, size_t *out_inlier_count);



/**
 * @brief [en] Removes the dominant background plane(s) from a preprocessed cloud.
 * @brief [zh] 从预处理点云中移除主背景平面（及可选的次级平面）。
 * @param cloud [en] Preprocessed points in metres.
 * @param cloud [zh] 以米为单位的预处理点。
 * @param cfg [en] Measurement configuration.
 * @param cfg [zh] 测量配置。
 * @param out_remaining [en] Receives the remaining points, or NULL on failure.
 * @param out_remaining [zh] 接收剩余点；失败时为 NULL。
 * @return [en] kSuccess or kInvalidConfig.
 * @return [zh] kSuccess 或 kInvalidConfig。
 * @since 2.0
 */
vm_status_t vm_remove_dominant_plane(const vm_cloud_t *cloud, const vm_config_t *cfg, vm_owned_cloud_t **out_remaining);



/**
 * @brief [en] Builds the empty-oven baseline model from one or more baseline frames.
 * @brief [zh] 从一帧或多帧空炉点云建立空炉基线模型。
 * @param baseline_frames [en] Array of empty-oven clouds in `cfg->input_unit`.
 * @param baseline_frames [zh] 以 `cfg->input_unit` 为单位的空炉点云数组。
 * @param frame_count [en] Number of baseline frames.
 * @param frame_count [zh] 空炉帧数。
 * @param orientation_points [en] Food-side points used to orient the plane normal; may be NULL.
 * @param orientation_points [zh] 用于确定平面法向的食材侧点；可为 NULL。
 * @param cfg [en] Measurement configuration.
 * @param cfg [zh] 测量配置。
 * @param out_baseline [en] Receives the built baseline, or NULL on failure.
 * @param out_baseline [zh] 接收构建好的基线；失败时为 NULL。
 * @return [en] kSuccess, kEmptyBaseline, kNonFiniteBaseline, kInvalidConfig, kPlaneNotFound, or kBaselineNoCells.
 * @return [zh] kSuccess、kEmptyBaseline、kNonFiniteBaseline、kInvalidConfig、kPlaneNotFound 或 kBaselineNoCells。
 * @since 2.0
 */
vm_status_t vm_build_baseline_model(const vm_cloud_t *baseline_frames, size_t frame_count,
                                    const vm_cloud_t *orientation_points, const vm_config_t *cfg,
                                    vm_baseline_t **out_baseline);



/**
 * @brief [en] Returns the number of baseline frames.
 * @brief [zh] 返回空炉基线帧数。
 * @since 2.0
 */
size_t vm_baseline_frame_count(const vm_baseline_t *baseline);



/**
 * @brief [en] Returns the number of baseline heightmap cells.
 * @brief [zh] 返回基线高度图单元数。
 * @since 2.0
 */
size_t vm_baseline_cell_count(const vm_baseline_t *baseline);



/**
 * @brief [en] Releases a baseline model.
 * @brief [zh] 释放基线模型。
 * @since 2.0
 */
void vm_baseline_destroy(vm_baseline_t *baseline);



/**
 * @brief [en] Filters food points by baseline-relative height, clusters them in the baseline plane, and selects components.
 * @brief [zh] 按相对基线高度过滤食材点，在基准面内聚类并选择组件。
 * @param food [en] Food points in metres after background-plane removal.
 * @param food [zh] 移除背景平面后以米为单位的食材点。
 * @param baseline [en] Reusable empty-oven baseline model.
 * @param baseline [zh] 可复用的空炉基线模型。
 * @param cfg [en] Measurement configuration.
 * @param cfg [zh] 测量配置。
 * @param out_components [en] Receives the selected components, or NULL on failure.
 * @param out_components [zh] 接收选中的组件；失败时为 NULL。
 * @return [en] kSuccess, kInsufficientCoverage, kFoodNotFound, or kInvalidConfig.
 * @return [zh] kSuccess、kInsufficientCoverage、kFoodNotFound 或 kInvalidConfig。
 * @since 2.0
 */
vm_status_t vm_extract_food_components(const vm_cloud_t *food, const vm_baseline_t *baseline, const vm_config_t *cfg,
                                       vm_components_t **out_components);



/**
 * @brief [en] Returns the total number of non-noise DBSCAN clusters found before selection.
 * @brief [zh] 返回选择前发现的非噪声 DBSCAN 簇总数。
 * @since 2.0
 */
size_t vm_components_cluster_count(const vm_components_t *components);



/**
 * @brief [en] Returns the number of selected components.
 * @brief [zh] 返回选中组件数。
 * @since 2.0
 */
size_t vm_components_count(const vm_components_t *components);



/**
 * @brief [en] Releases a component set.
 * @brief [zh] 释放组件集合。
 * @since 2.0
 */
void vm_components_destroy(vm_components_t *components);



/**
 * @brief [en] Measures component volume by integrating baseline-relative heights with conservative hole completion.
 * @brief [zh] 通过积分相对基线高度并保守补洞来测量组件体积。
 * @param components [en] Selected food components produced by `vm_extract_food_components`.
 * @param components [zh] 由 `vm_extract_food_components` 产生的选中组件。
 * @param baseline [en] Reusable empty-oven baseline model.
 * @param baseline [zh] 可复用的空炉基线模型。
 * @param cfg [en] Measurement configuration.
 * @param cfg [zh] 测量配置。
 * @param out_volume [en] Receives the integrated volume and diagnostics.
 * @param out_volume [zh] 接收积分体积与诊断信息。
 * @return [en] kSuccess, kInvalidConfig, kInsufficientCoverage, or kFoodNotFound.
 * @return [zh] kSuccess、kInvalidConfig、kInsufficientCoverage 或 kFoodNotFound。
 * @since 2.0
 */
vm_status_t vm_measure_component_volume(const vm_components_t *components, const vm_baseline_t *baseline,
                                        const vm_config_t *cfg, vm_component_volume_t *out_volume);



/**
 * @brief [en] Creates an end-to-end PCD-IM volume pipeline.
 * @brief [zh] 创建端到端的 PCD-IM 体积流水线。
 * @return [en] A new pipeline, or NULL on allocation failure.
 * @return [zh] 新流水线；分配失败时返回 NULL。
 * @since 2.0
 */
vm_pipeline_t *vm_pipeline_create(void);



/**
 * @brief [en] Releases a volume pipeline.
 * @brief [zh] 释放体积流水线。
 * @param pipeline [en] The pipeline to release; may be NULL.
 * @param pipeline [zh] 待释放的流水线；可为 NULL。
 * @since 2.0
 */
void vm_pipeline_destroy(vm_pipeline_t *pipeline);



/**
 * @brief [en] Measures food volume in cubic centimeters from empty-oven baseline frames and a food frame.
 * @brief [zh] 从空炉基线帧与食材帧测量食材体积（立方厘米）。
 * @param pipeline [en] A pipeline created by `vm_pipeline_create`; must not be NULL.
 * @param pipeline [zh] 由 `vm_pipeline_create` 创建的流水线；不能为 NULL。
 * @param baseline_frames [en] Array of empty-oven clouds in `cfg->input_unit`.
 * @param baseline_frames [zh] 以 `cfg->input_unit` 为单位的空炉点云数组。
 * @param baseline_count [en] Number of baseline frames.
 * @param baseline_count [zh] 空炉帧数。
 * @param food [en] The food point cloud in `cfg->input_unit`.
 * @param food [zh] 以 `cfg->input_unit` 为单位的食材点云。
 * @param cfg [en] Measurement configuration; NULL uses defaults.
 * @param cfg [zh] 测量配置；NULL 表示使用默认值。
 * @param out_estimate [en] Receives the volume estimate.
 * @param out_estimate [zh] 接收体积估计结果。
 * @return [en] The measurement status; out_estimate->status mirrors it on success.
 * @return [zh] 测量状态；成功时 out_estimate->status 与之相同。
 * @since 2.0
 */
vm_status_t vm_measure(vm_pipeline_t *pipeline, const vm_cloud_t *baseline_frames, size_t baseline_count,
                       const vm_cloud_t *food, const vm_config_t *cfg, vm_estimate_t *out_estimate);



/**
 * @brief [en] Logging severity levels.
 * @brief [zh] 日志严重级别。
 * @since 2.0
 */
typedef enum vm_log_level {
    VM_LOG_TRACE = 0,
    VM_LOG_DEBUG = 1,
    VM_LOG_INFO = 2,
    VM_LOG_WARNING = 3,
    VM_LOG_ERROR = 4,
    VM_LOG_OFF = 5
} vm_log_level_t;



/**
 * @brief [en] Write mode of the log file sink.
 * @brief [zh] 日志文件 sink 的写入模式。
 * @since 2.0
 */
typedef enum vm_log_file_mode { VM_LOG_FILE_APPEND = 0, VM_LOG_FILE_TRUNCATE = 1 } vm_log_file_mode_t;



/**
 * @brief [en] Sets the minimum emitted log level; messages below it are dropped.
 * @brief [zh] 设置要输出的最小日志级别；低于该级别的消息被丢弃。
 * @since 2.0
 */
void vm_log_set_level(vm_log_level_t level);



/**
 * @brief [en] Returns the current minimum emitted log level.
 * @brief [zh] 返回当前要输出的最低日志级别。
 * @since 2.0
 */
vm_log_level_t vm_log_get_level(void);



/**
 * @brief [en] Enables or disables console output.
 * @brief [zh] 启用或禁用控制台输出。
 * @since 2.0
 */
void vm_log_set_console(int enabled);



/**
 * @brief [en] Opens a log file; an empty path closes the current file.
 * @brief [zh] 打开日志文件；空路径则关闭当前文件。
 * @since 2.0
 */
void vm_log_set_file(const char *path, vm_log_file_mode_t mode);



/**
 * @brief [en] Writes one log line when the level is at or above the configured minimum.
 * @brief [zh] 当级别不低于配置的最小级别时写入一行日志。
 * @since 2.0
 */
void vm_log_write(vm_log_level_t level, const char *message);



#ifdef __cplusplus
}
#endif

#endif
