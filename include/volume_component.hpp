// Conan::ImportStart
#pragma once
#include <vector>
#include "volume_baseline.hpp"
#include "volume_types.hpp"
// Conan::ImportEnd



namespace vm {



/**
 * @brief [en] Selected foreground food components: stable public labels plus their point clouds.
 * @brief [zh] 选中的前景食材组件：稳定的公开标签及其点云。
 * @exporter
 */
struct FoodComponents {
    std::vector<int> labels;        // ascending, one entry per component
    std::vector<PointCloud> clouds; // one cloud per label, same order as `labels`
};



/**
 * @brief [en] Filters food points by baseline-relative height, clusters them in the baseline plane, and selects components.
 * @brief [zh] 按相对基线高度过滤食材点，在基准面内聚类并选择组件。
 * @param food_m [en] Food points in metres after background-plane removal.
 * @param food_m [zh] 移除背景平面后以米为单位的食材点。
 * @param baseline [en] Reusable empty-oven baseline model.
 * @param baseline [zh] 可复用的空炉基线模型。
 * @param cfg [en] Measurement configuration.
 * @param cfg [zh] 测量配置。
 * @param out [en] Receives the selected component labels and their point clouds.
 * @param out [zh] 接收选中的组件标签及其点云。
 * @return [en] kSuccess, kInsufficientCoverage, kFoodNotFound, or kInvalidConfig.
 * @return [zh] kSuccess、kInsufficientCoverage、kFoodNotFound 或 kInvalidConfig。
 * @exporter
 */
MeasurementStatus extract_food_components(const PointCloud& food_m, const BaselineModel& baseline,
                                          const MeasurementConfig& cfg, FoodComponents& out);



} // namespace vm