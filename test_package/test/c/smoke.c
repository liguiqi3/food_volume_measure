#include "volume_c.h"
#include <stdio.h>
#include <stdlib.h>

#define BASELINE_HALF 0.15
#define CELL 0.005
#define CUBOID_TOP 0.02
#define CUBOID_MIN 0.0025
#define CUBOID_MAX 0.0975
#define POINT_CAPACITY 5000

int main(void) {
    vm_point3f_t *baseline = (vm_point3f_t *)malloc(POINT_CAPACITY * sizeof(vm_point3f_t));
    vm_point3f_t *food = (vm_point3f_t *)malloc(POINT_CAPACITY * sizeof(vm_point3f_t));
    if (baseline == NULL || food == NULL) {
        free(baseline);
        free(food);
        return 1;
    }

    /* Flat z=0 baseline grid; the food cloud reuses it plus a raised cuboid. */
    size_t baseline_count = 0;
    size_t food_count = 0;
    for (double x = -BASELINE_HALF; x <= BASELINE_HALF + 1.0e-9; x += CELL) {
        for (double y = -BASELINE_HALF; y <= BASELINE_HALF + 1.0e-9; y += CELL) {
            baseline[baseline_count].x = (float)x;
            baseline[baseline_count].y = (float)y;
            baseline[baseline_count].z = 0.0F;
            food[food_count].x = (float)x;
            food[food_count].y = (float)y;
            food[food_count].z = 0.0F;
            ++baseline_count;
            ++food_count;
        }
    }
    for (double x = CUBOID_MIN; x <= CUBOID_MAX + 1.0e-9; x += CELL) {
        for (double y = CUBOID_MIN; y <= CUBOID_MAX + 1.0e-9; y += CELL) {
            food[food_count].x = (float)x;
            food[food_count].y = (float)y;
            food[food_count].z = (float)CUBOID_TOP;
            ++food_count;
        }
    }

    vm_config_t cfg;
    vm_config_init(&cfg);
    cfg.voxel_size_m = 0.002;
    cfg.integration_resolution_m = 0.005;

    vm_pipeline_t *pipeline = vm_pipeline_create();
    if (pipeline == NULL) {
        free(baseline);
        free(food);
        return 1;
    }

    vm_cloud_t baseline_cloud = {baseline, baseline_count};
    vm_cloud_t food_cloud = {food, food_count};
    vm_estimate_t estimate;
    const vm_status_t status = vm_measure(pipeline, &baseline_cloud, 1, &food_cloud, &cfg, &estimate);

    printf("C smoke: status=%s volume_cm3=%.3f components=%zu\n", vm_status_to_string(estimate.status),
           estimate.volume_cm3, estimate.component_count);

    vm_pipeline_destroy(pipeline);
    free(baseline);
    free(food);

    return (status == VM_STATUS_SUCCESS && estimate.volume_cm3 > 100.0) ? 0 : 1;
}
