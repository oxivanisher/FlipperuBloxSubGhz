#include "config.h"

#include <furi.h>
#include <storage/storage.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

void config_defaults(AppConfig* cfg) {
    cfg->target_lat       = 0;
    cfg->target_lon       = 0;
    cfg->target_set       = false;
    cfg->radius_m         = CONFIG_RADIUS_DEFAULT_M;
    cfg->cooldown_s       = CONFIG_COOLDOWN_DEFAULT_S;
    cfg->subghz_file[0]   = '\0';
    cfg->tracking_enabled = false;
}

void config_load(AppConfig* cfg) {
    config_defaults(cfg);

    Storage* storage = furi_record_open(RECORD_STORAGE);
    File*    file    = storage_file_alloc(storage);

    if(!storage_file_open(file, CONFIG_PATH, FSAM_READ, FSOM_OPEN_EXISTING)) {
        storage_file_free(file);
        furi_record_close(RECORD_STORAGE);
        return;
    }

    /* Read entire config file (it is always small, < 512 B) */
    uint64_t file_size = storage_file_size(file);
    if(file_size == 0 || file_size > 512) {
        storage_file_close(file);
        storage_file_free(file);
        furi_record_close(RECORD_STORAGE);
        return;
    }

    char buf[513];
    size_t read = storage_file_read(file, buf, (uint16_t)file_size);
    buf[read] = '\0';

    storage_file_close(file);
    storage_file_free(file);
    furi_record_close(RECORD_STORAGE);

    /* Parse line by line without strtok_r (disabled in Flipper SDK) */
    char* p = buf;
    while(*p) {
        /* Find end of this line */
        char* end = p;
        while(*end && *end != '\n') end++;

        /* Copy and strip CR */
        char   line[160];
        size_t len = (size_t)(end - p);
        if(len >= sizeof(line)) len = sizeof(line) - 1;
        memcpy(line, p, len);
        line[len] = '\0';
        while(len > 0 && line[len - 1] == '\r') line[--len] = '\0';

        /* Advance past the newline */
        p = *end ? end + 1 : end;

        if(strncmp(line, "lat=", 4) == 0) {
            cfg->target_lat = strtod(line + 4, NULL);
            cfg->target_set = true;
        } else if(strncmp(line, "lon=", 4) == 0) {
            cfg->target_lon = strtod(line + 4, NULL);
        } else if(strncmp(line, "radius=", 7) == 0) {
            cfg->radius_m = strtof(line + 7, NULL);
        } else if(strncmp(line, "cooldown=", 9) == 0) {
            cfg->cooldown_s = (uint32_t)atoi(line + 9);
        } else if(strncmp(line, "file=", 5) == 0) {
            strncpy(cfg->subghz_file, line + 5, CONFIG_SUB_PATH_MAX - 1);
            cfg->subghz_file[CONFIG_SUB_PATH_MAX - 1] = '\0';
        } else if(strncmp(line, "tracking=", 9) == 0) {
            cfg->tracking_enabled = (line[9] == '1');
        }
    }
}

bool config_save(const AppConfig* cfg) {
    Storage* storage = furi_record_open(RECORD_STORAGE);
    storage_simply_mkdir(storage, CONFIG_DIR);

    File* file = storage_file_alloc(storage);
    if(!storage_file_open(file, CONFIG_PATH, FSAM_WRITE, FSOM_CREATE_ALWAYS)) {
        storage_file_free(file);
        furi_record_close(RECORD_STORAGE);
        return false;
    }

    char buf[320];
    int  n = snprintf(
        buf,
        sizeof(buf),
        "lat=%.8f\nlon=%.8f\nradius=%.1f\ncooldown=%lu\nfile=%s\ntracking=%d\n",
        cfg->target_lat,
        cfg->target_lon,
        (double)cfg->radius_m,
        (unsigned long)cfg->cooldown_s,
        cfg->subghz_file,
        cfg->tracking_enabled ? 1 : 0);

    if(n > 0) storage_file_write(file, buf, (uint16_t)n);

    storage_file_close(file);
    storage_file_free(file);
    furi_record_close(RECORD_STORAGE);
    return true;
}
