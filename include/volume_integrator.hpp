// Conan::ImportStart
#pragma once
#include <cstddef>
#include <vector>
#include "volume_baseline.hpp"
#include "volume_component.hpp"
#include "volume_types.hpp"
// Conan::ImportEnd



namespace vm {



/**
 * @brief [en] Per-component PCD-IM integration result.
 * @brief [zh] 单组件 PCD-IM 积分结果。
 * @exporter
 */
struct ComponentVolumeEstimate {
    double raw_volume_cm3 = 0.0;
    double interpolated_volume_cm3 = 0.0;
    double volume_cm3 = 0.0;

    std::size_t top_surface_points = 0;
    std::size_t measured_cells = 0;
    std::size_t interpolated_cells = 0;
    std::size_t occupied_cells = 0;
    std::size_t bbox_cell_count = 0;
    std::size_t missing_baseline_cells = 0;
    std::size_t unfilled_hole_cells = 0;

    double footprint_area_m2 = 0.0;
    double coverage_ratio = 0.0;
    double mean_height_m = 0.0;
    double max_height_m = 0.0;
};



/**
 * @brief [en] Per-component integration results, aligned with `FoodComponents.labels`.
 * @brief [zh] 逐组件积分结果，与 `FoodComponents.labels` 一一对应。
 * @exporter
 */
struct ComponentVolumeResults {
    std::vector<int> labels;
    std::vector<ComponentVolumeEstimate> estimates;
};



/**
 * @brief [en] Measures component volume by integrating baseline-relative heights with conservative hole completion.
 * @brief [zh] 通过积分相对基线高度并保守补洞来测量组件体积。
 * @param components [en] Selected food components produced by `extract_food_components`.
 * @param components [zh] 由 `extract_food_components` 产生的选中食材组件。
 * @param baseline [en] Reusable empty-oven baseline model.
 * @param baseline [zh] 可复用的空炉基线模型。
 * @param cfg [en] Measurement configuration.
 * @param cfg [zh] 测量配置。
 * @param out [en] Receives the integrated volume and diagnostics.
 * @param out [zh] 接收积分体积与诊断信息。
 * @return [en] kSuccess, kInvalidConfig, kInsufficientCoverage, or kFoodNotFound.
 * @return [zh] kSuccess、kInvalidConfig、kInsufficientCoverage 或 kFoodNotFound。
 * @exporter
 */
MeasurementStatus measure_component_volume(const FoodComponents& components, const BaselineModel& baseline,
                                           const MeasurementConfig& cfg, ComponentVolumeEstimate& out);



/**
 * @brief [en] Measures the volume of each selected component independently.
 * @brief [zh] 独立测量每个选中组件的体积。
 * @param components [en] Selected food components produced by `extract_food_components`.
 * @param components [zh] 由 `extract_food_components` 产生的选中食材组件。
 * @param baseline [en] Reusable empty-oven baseline model.
 * @param baseline [zh] 可复用的空炉基线模型。
 * @param cfg [en] Measurement configuration.
 * @param cfg [zh] 测量配置。
 * @param out [en] Receives one integration result per component, in the same order as `components.labels`.
 * @param out [zh] 接收每个组件各一条积分结果，顺序与 `components.labels` 一致。
 * @return [en] kSuccess, or the first per-component failure status.
 * @return [zh] kSuccess，或首个失败组件的状态。
 * @exporter
 */
MeasurementStatus measure_component_volumes(const FoodComponents& components, const BaselineModel& baseline,
                                            const MeasurementConfig& cfg, ComponentVolumeResults& out);



} // namespace vm
