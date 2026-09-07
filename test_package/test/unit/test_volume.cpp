#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <vector>
#include "volume_baseline.hpp"
#include "volume_component.hpp"
#include "volume_integrator.hpp"
#include "volume_pipeline.hpp"
#include "volume_pointcloudprocess.hpp"



namespace {

constexpr double kCell = 0.005;
constexpr double kCuboidTop = 0.02;
constexpr double kExpectedVolumeCm3 = 200.0;

vm::PointCloud make_baseline(double half = 0.15) {
    vm::PointCloud cloud;
    for (double x = -half; x <= half + 1.0e-9; x += kCell) {
        for (double y = -half; y <= half + 1.0e-9; y += kCell) {
            cloud.points.push_back({static_cast<float>(x), static_cast<float>(y), 0.0F});
        }
    }
    return cloud;
}

vm::PointCloud make_food(double u_min = 0.0025, double u_max = 0.0975, double v_min = 0.0025, double v_max = 0.0975,
                         double hole_u = -1.0, double hole_v = -1.0) {
    vm::PointCloud cloud = make_baseline();
    for (double x = u_min; x <= u_max + 1.0e-9; x += kCell) {
        for (double y = v_min; y <= v_max + 1.0e-9; y += kCell) {
            if (std::fabs(x - hole_u) < 1.0e-6 && std::fabs(y - hole_v) < 1.0e-6) {
                continue;
            }
            cloud.points.push_back({static_cast<float>(x), static_cast<float>(y), static_cast<float>(kCuboidTop)});
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



TEST(VolumeTypes, LengthUnitScale) {
    EXPECT_DOUBLE_EQ(vm::length_unit_to_meter_scale(vm::LengthUnit::kMeter), 1.0);
    EXPECT_DOUBLE_EQ(vm::length_unit_to_meter_scale(vm::LengthUnit::kMillimeter), 0.001);
}



TEST(VolumeTypes, FiniteCheck) {
    const vm::Point3f ok{1.0F, 2.0F, 3.0F};
    EXPECT_TRUE(vm::is_finite(ok));
    const vm::Point3f bad{1.0F, std::numeric_limits<float>::quiet_NaN(), 3.0F};
    EXPECT_FALSE(vm::is_finite(bad));
}



TEST(VolumePipeline, EmptyBaseline) {
    vm::VolumePipeline pipeline;
    const auto est = pipeline.measure({}, make_food(), make_config());
    EXPECT_EQ(est.status, vm::MeasurementStatus::kEmptyBaseline);
}



TEST(VolumePipeline, NonFiniteFoodPointsSkipped) {
    // A non-finite depth return must be dropped, not fatal, as long as valid food remains.
    vm::PointCloud cloud = make_food();
    cloud.points.push_back({0.0F, std::numeric_limits<float>::infinity(), 0.0F});
    vm::VolumePipeline pipeline;
    const auto est = pipeline.measure({make_baseline()}, cloud, make_config());
    EXPECT_EQ(est.status, vm::MeasurementStatus::kSuccess) << est.message;
}



TEST(VolumePipeline, AllNonFiniteFoodInput) {
    vm::PointCloud cloud;
    cloud.points.push_back({0.0F, std::numeric_limits<float>::infinity(), 0.0F});
    cloud.points.push_back({std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F});
    vm::VolumePipeline pipeline;
    const auto est = pipeline.measure({make_baseline()}, cloud, make_config());
    EXPECT_EQ(est.status, vm::MeasurementStatus::kNonFiniteInput);
}



TEST(VolumePipeline, CuboidEndToEnd) {
    const std::vector<vm::PointCloud> baselines{make_baseline()};
    vm::VolumePipeline pipeline;
    const auto est = pipeline.measure(baselines, make_food(), make_config());
    ASSERT_EQ(est.status, vm::MeasurementStatus::kSuccess) << est.message;
    EXPECT_NEAR(est.volume_cm3, kExpectedVolumeCm3, 10.0);
    EXPECT_GT(est.coverage_ratio, 0.8);
    EXPECT_EQ(est.baseline_frames, 1u);
    EXPECT_GT(est.baseline_cell_count, 0u);
}



TEST(VolumePipeline, EmptyTrayNoFood) {
    const std::vector<vm::PointCloud> baselines{make_baseline()};
    vm::VolumePipeline pipeline;
    const auto est = pipeline.measure(baselines, make_baseline(), make_config());
    EXPECT_EQ(est.status, vm::MeasurementStatus::kInsufficientCoverage);
}



TEST(VolumePipeline, MillimeterInput) {
    auto baseline = make_baseline();
    auto food = make_food();
    for (auto& cloud : std::vector<vm::PointCloud*>{&baseline, &food}) {
        for (auto& p : cloud->points) {
            p.x *= 1000.0F;
            p.y *= 1000.0F;
            p.z *= 1000.0F;
        }
    }
    auto cfg = make_config();
    cfg.input_unit = vm::LengthUnit::kMillimeter;
    vm::VolumePipeline pipeline;
    const auto est = pipeline.measure({baseline}, food, cfg);
    ASSERT_EQ(est.status, vm::MeasurementStatus::kSuccess) << est.message;
    EXPECT_NEAR(est.volume_cm3, kExpectedVolumeCm3, 10.0);
}



TEST(VolumePipeline, TiltedTrayDetected) {
    const double theta = 0.5;
    const double s = std::sin(theta);
    const double c = std::cos(theta);

    const auto mk = [&](double u, double v, double h) -> vm::Point3f {
        return {static_cast<float>(u * c - h * s), static_cast<float>(v), static_cast<float>(u * s + h * c)};
    };

    vm::PointCloud baseline;
    for (double u = -0.15; u <= 0.15 + 1.0e-9; u += kCell) {
        for (double v = -0.15; v <= 0.15 + 1.0e-9; v += kCell) {
            baseline.points.push_back(mk(u, v, 0.0));
        }
    }
    vm::PointCloud food = baseline;
    for (double u = 0.0025; u <= 0.0975 + 1.0e-9; u += kCell) {
        for (double v = 0.0025; v <= 0.0975 + 1.0e-9; v += kCell) {
            food.points.push_back(mk(u, v, kCuboidTop));
        }
    }

    vm::VolumePipeline pipeline;
    const auto est = pipeline.measure({baseline}, food, make_config());
    ASSERT_EQ(est.status, vm::MeasurementStatus::kSuccess) << est.message;
    EXPECT_NEAR(est.volume_cm3, kExpectedVolumeCm3, 25.0);
}



TEST(VolumePipeline, MultiComponentVolume) {
    const std::vector<vm::PointCloud> baselines{make_baseline(0.25)};
    vm::PointCloud food = make_baseline(0.25);
    // Two separated cuboids; the second starts far enough to form its own footprint cluster.
    for (double x = 0.0025; x <= 0.0975 + 1.0e-9; x += kCell) {
        for (double y = 0.0025; y <= 0.0975 + 1.0e-9; y += kCell) {
            food.points.push_back({static_cast<float>(x), static_cast<float>(y), static_cast<float>(kCuboidTop)});
        }
    }
    for (double x = 0.1125; x <= 0.2075 + 1.0e-9; x += kCell) {
        for (double y = 0.0025; y <= 0.0975 + 1.0e-9; y += kCell) {
            food.points.push_back({static_cast<float>(x), static_cast<float>(y), static_cast<float>(kCuboidTop)});
        }
    }

    vm::VolumePipeline pipeline;
    const auto est = pipeline.measure(baselines, food, make_config());
    ASSERT_EQ(est.status, vm::MeasurementStatus::kSuccess) << est.message;
    EXPECT_EQ(est.component_count, 2u);
    EXPECT_NEAR(est.volume_cm3, 2.0 * kExpectedVolumeCm3, 20.0);
}



TEST(VolumePipeline, SmallHoleInterpolated) {
    const std::vector<vm::PointCloud> baselines{make_baseline()};
    // One missing interior cell (0.0525, 0.0525) surrounded by measured cells.
    vm::VolumePipeline pipeline;
    const auto est =
        pipeline.measure(baselines, make_food(0.0025, 0.0975, 0.0025, 0.0975, 0.0525, 0.0525), make_config());
    ASSERT_EQ(est.status, vm::MeasurementStatus::kSuccess) << est.message;
    EXPECT_NEAR(est.volume_cm3, kExpectedVolumeCm3, 10.0);
    EXPECT_EQ(est.interpolated_cells, 1u);
    EXPECT_NEAR(est.interpolated_volume_cm3, kCuboidTop * kCell * kCell * 1.0e6, 0.05);
}



TEST(Preprocess, EmptyInput) {
    vm::PreprocessResult out;
    EXPECT_EQ(vm::preprocess_cloud(vm::PointCloud{}, make_config(), out), vm::MeasurementStatus::kEmptyInput);
}



TEST(Preprocess, NonFiniteInput) {
    vm::PointCloud cloud;
    cloud.points.push_back({0.0F, std::numeric_limits<float>::infinity(), 0.0F});
    vm::PreprocessResult out;
    EXPECT_EQ(vm::preprocess_cloud(cloud, make_config(), out), vm::MeasurementStatus::kNonFiniteInput);
}



TEST(Preprocess, UnitNormalizeAndVoxel) {
    vm::PointCloud in;
    in.points.push_back({0.0F, 0.0F, 0.0F});
    in.points.push_back({1.0F, 0.0F, 0.0F}); // 1 mm -> 0.001 m
    auto cfg = make_config();
    cfg.input_unit = vm::LengthUnit::kMillimeter;
    cfg.voxel_size_m = 0.002F;

    vm::PreprocessResult out;
    ASSERT_EQ(vm::preprocess_cloud(in, cfg, out), vm::MeasurementStatus::kSuccess);
    EXPECT_EQ(out.input_points, 2u);
    EXPECT_EQ(out.retained_points, out.cloud.points.size());
    // Both points are finite, so retained points must be positive.
    EXPECT_GT(out.retained_points, 0u);
}



TEST(Plane, FitHorizontalPlane) {
    vm::PointCloud cloud;
    for (float x = -0.1F; x <= 0.1F; x += 0.01F) {
        for (float y = -0.1F; y <= 0.1F; y += 0.01F) {
            cloud.points.push_back({x, y, 0.0F});
        }
    }
    vm::Plane plane;
    std::vector<std::size_t> inliers;
    ASSERT_EQ(vm::fit_plane_ransac(cloud, 0.001, 100, plane, inliers), vm::MeasurementStatus::kSuccess);
    EXPECT_NEAR(std::fabs(plane.nz), 1.0F, 0.05F);
    EXPECT_NEAR(plane.d, 0.0, 0.001);
    EXPECT_GT(inliers.size(), 0u);
}



TEST(Plane, RemoveDominantPlane) {
    vm::PointCloud cloud;
    for (float x = -0.1F; x <= 0.1F; x += 0.01F) {
        for (float y = -0.1F; y <= 0.1F; y += 0.01F) {
            cloud.points.push_back({x, y, 0.0F});
        }
    }
    cloud.points.push_back({0.0F, 0.0F, 0.02F});

    vm::PointCloud remaining;
    ASSERT_EQ(vm::remove_dominant_plane(cloud, make_config(), remaining), vm::MeasurementStatus::kSuccess);
    ASSERT_FALSE(remaining.points.empty());
    for (const auto& p : remaining.points) {
        EXPECT_NEAR(p.z, 0.02F, 1.0e-4F);
    }
}



TEST(Dbscan, ClustersAndNoise) {
    std::vector<vm::Point3f> points;
    points.push_back({0.0F, 0.0F, 0.0F});
    points.push_back({0.01F, 0.0F, 0.0F});
    points.push_back({0.02F, 0.0F, 0.0F});
    points.push_back({1.0F, 1.0F, 1.0F});
    points.push_back({1.01F, 1.0F, 1.0F});
    points.push_back({1.02F, 1.0F, 1.0F});
    points.push_back({5.0F, 5.0F, 5.0F});

    const std::vector<int> labels = vm::dbscan_labels(points, 0.05, 2);
    ASSERT_EQ(labels.size(), points.size());
    EXPECT_EQ(labels[0], labels[1]);
    EXPECT_EQ(labels[1], labels[2]);
    EXPECT_EQ(labels[3], labels[4]);
    EXPECT_EQ(labels[4], labels[5]);
    EXPECT_NE(labels[0], labels[3]);
    EXPECT_EQ(labels[6], -1);
}



TEST(Baseline, EmptyBaseline) {
    vm::BaselineModel model;
    EXPECT_EQ(vm::build_baseline_model({}, {}, make_config(), model), vm::MeasurementStatus::kEmptyBaseline);
}



TEST(Baseline, BuildBaselineModel) {
    vm::BaselineModel model;
    const std::vector<vm::PointCloud> frames{make_baseline()};
    ASSERT_EQ(vm::build_baseline_model(frames, {}, make_config(), model), vm::MeasurementStatus::kSuccess);
    EXPECT_EQ(model.frame_count, 1u);
    EXPECT_GT(model.cell_count, 0u);
    EXPECT_NE(model.data, nullptr);
}



TEST(Atomic, ComponentExtractionAndVolume) {
    const std::vector<vm::PointCloud> frames{make_baseline()};

    vm::PreprocessResult pre;
    ASSERT_EQ(vm::preprocess_cloud(make_food(), make_config(), pre), vm::MeasurementStatus::kSuccess);

    vm::PointCloud remaining;
    ASSERT_EQ(vm::remove_dominant_plane(pre.cloud, make_config(), remaining), vm::MeasurementStatus::kSuccess);

    vm::BaselineModel baseline;
    ASSERT_EQ(vm::build_baseline_model(frames, remaining, make_config(), baseline), vm::MeasurementStatus::kSuccess);

    vm::FoodComponents components;
    ASSERT_EQ(vm::extract_food_components(remaining, baseline, make_config(), components),
              vm::MeasurementStatus::kSuccess);
    EXPECT_FALSE(components.labels.empty());
    EXPECT_EQ(components.labels.size(), components.clouds.size());

    vm::ComponentVolumeEstimate volume;
    ASSERT_EQ(vm::measure_component_volume(components, baseline, make_config(), volume),
              vm::MeasurementStatus::kSuccess);
    EXPECT_NEAR(volume.volume_cm3, kExpectedVolumeCm3, 10.0);
    EXPECT_GT(volume.measured_cells, 0u);
}
