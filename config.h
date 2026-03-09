#pragma once

#include <stdbool.h>
#include <stdint.h>

#define CONFIG_DIR  "/ext/apps_data/gps_garage"
#define CONFIG_PATH "/ext/apps_data/gps_garage/config.txt"

#define CONFIG_SUB_PATH_MAX  128u
#define CONFIG_RADIUS_MIN_M  5.0f
#define CONFIG_RADIUS_MAX_M  5000.0f
#define CONFIG_RADIUS_STEP_M 5.0f
#define CONFIG_RADIUS_DEFAULT_M 50.0f
#define CONFIG_COOLDOWN_DEFAULT_S 30u

typedef struct {
    double   target_lat;
    double   target_lon;
    bool     target_set;
    float    radius_m;
    uint32_t cooldown_s;
    char     subghz_file[CONFIG_SUB_PATH_MAX];
    bool     tracking_enabled;
} AppConfig;

void config_defaults(AppConfig* cfg);
void config_load(AppConfig* cfg);
bool config_save(const AppConfig* cfg);
