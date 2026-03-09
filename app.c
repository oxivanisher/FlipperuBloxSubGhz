/**
 * GPS Garage Opener — main application
 *
 * Hardware wiring:
 *   uBlox GPS module TX  →  Flipper GPIO pin 14  (USART1 RX)
 *   uBlox GPS module RX  ←  Flipper GPIO pin 13  (USART1 TX, optional)
 *   uBlox GPS module VCC ←  Flipper 3V3 pin
 *   uBlox GPS module GND ←  Flipper GND pin
 *
 * Controls:
 *   OK (short)    – manual SubGHz transmit
 *   OK (long)     – toggle auto-tracking on/off
 *   UP (long)     – capture current GPS position as trigger target
 *   DOWN (long)   – browse and select .sub file
 *   LEFT / RIGHT  – decrease / increase trigger radius by 5 m
 *   BACK          – exit
 */

#include "app.h"
#include "gps.h"
#include "geo.h"
#include "config.h"

#include <furi.h>
#include <furi_hal.h>
#include <gui/gui.h>
#include <gui/view.h>
#include <gui/view_dispatcher.h>
#include <gui/canvas.h>
#include <input/input.h>
#include <dialogs/dialogs.h>
#include <storage/storage.h>
#include <flipper_format/flipper_format.h>
/* SubGHz headers — note the filenames changed in this firmware version */
#include <lib/subghz/environment.h>
#include <lib/subghz/transmitter.h>
#include <lib/subghz/devices/devices.h>
#include <lib/subghz/devices/preset.h>
#include <lib/subghz/devices/cc1101_int/cc1101_int_interconnect.h>
#include <lib/subghz/protocols/protocol_items.h>

#include <stdio.h>
#include <string.h>

/* ══════════════════════════════════════════════════════════════════════
 * View model — a thin wrapper so the draw callback can reach app state
 * ══════════════════════════════════════════════════════════════════════ */
typedef struct {
    GpsGarageApp* app;
} MainViewModel;

/* ══════════════════════════════════════════════════════════════════════
 * SubGHz transmission helpers
 * ══════════════════════════════════════════════════════════════════════ */

static FuriHalSubGhzPreset preset_from_name(const FuriString* name) {
    if(furi_string_equal_str(name, "FuriHalSubGhzPresetOok270Async"))
        return FuriHalSubGhzPresetOok270Async;
    if(furi_string_equal_str(name, "FuriHalSubGhzPreset2FSKDev238Async"))
        return FuriHalSubGhzPreset2FSKDev238Async;
    if(furi_string_equal_str(name, "FuriHalSubGhzPreset2FSKDev476Async"))
        return FuriHalSubGhzPreset2FSKDev476Async;
    if(furi_string_equal_str(name, "FuriHalSubGhzPresetMSK99_97KbAsync"))
        return FuriHalSubGhzPresetMSK99_97KbAsync;
    if(furi_string_equal_str(name, "FuriHalSubGhzPresetGFSK9_99KbAsync"))
        return FuriHalSubGhzPresetGFSK9_99KbAsync;
    if(furi_string_equal_str(name, "FuriHalSubGhzPresetCustom"))
        return FuriHalSubGhzPresetCustom;
    return FuriHalSubGhzPresetOok650Async; /* most common garage-door preset */
}

static bool subghz_transmit_file(const char* filepath) {
    Storage*       storage = furi_record_open(RECORD_STORAGE);
    FlipperFormat* fff     = flipper_format_file_alloc(storage);
    bool           ok      = false;

    do {
        if(!flipper_format_file_open_existing(fff, filepath)) break;

        FuriString* file_type = furi_string_alloc();
        uint32_t    version   = 0;
        bool        hdr_ok    = flipper_format_read_header(fff, file_type, &version);
        furi_string_free(file_type);
        if(!hdr_ok) break;

        uint32_t frequency = 433920000u;
        flipper_format_read_uint32(fff, "Frequency", &frequency, 1);

        FuriString* preset_str = furi_string_alloc();
        flipper_format_read_string(fff, "Preset", preset_str);
        FuriHalSubGhzPreset preset_enum = preset_from_name(preset_str);

        /* Read custom CC1101 preset registers if needed */
        uint8_t* custom_data     = NULL;
        uint32_t custom_data_len = 0;
        if(preset_enum == FuriHalSubGhzPresetCustom) {
            if(flipper_format_get_value_count(
                   fff, "Custom_preset_data", &custom_data_len) &&
               custom_data_len > 0) {
                custom_data = malloc(custom_data_len);
                if(custom_data) {
                    flipper_format_read_hex(
                        fff, "Custom_preset_data", custom_data, custom_data_len);
                }
            }
        }

        FuriString* proto_str = furi_string_alloc();
        flipper_format_read_string(fff, "Protocol", proto_str);

        /* 5 repeats — catches cases where the first trigger is at the edge of range */
        flipper_format_rewind(fff);
        uint32_t repeat = 5u;
        flipper_format_insert_or_update_uint32(fff, "Repeat", &repeat, 1);
        flipper_format_rewind(fff);

        /* Set up SubGHz environment and transmitter */
        SubGhzEnvironment* env = subghz_environment_alloc();
        subghz_environment_set_protocol_registry(
            env, &subghz_protocol_registry);

        SubGhzTransmitter* tx =
            subghz_transmitter_alloc_init(env, furi_string_get_cstr(proto_str));

        if(!tx) {
            subghz_environment_free(env);
            furi_string_free(proto_str);
            furi_string_free(preset_str);
            if(custom_data) free(custom_data);
            break;
        }

        if(subghz_transmitter_deserialize(tx, fff) != SubGhzProtocolStatusOk) {
            subghz_transmitter_free(tx);
            subghz_environment_free(env);
            furi_string_free(proto_str);
            furi_string_free(preset_str);
            if(custom_data) free(custom_data);
            break;
        }

        const SubGhzDevice* dev =
            subghz_devices_get_by_name(SUBGHZ_DEVICE_CC1101_INT_NAME);

        subghz_devices_reset(dev);
        subghz_devices_idle(dev);
        subghz_devices_load_preset(dev, preset_enum, custom_data);
        subghz_devices_set_frequency(dev, frequency);

        if(!subghz_devices_set_tx(dev)) {
            /* Frequency blocked by region settings */
            subghz_devices_idle(dev);
            subghz_transmitter_free(tx);
            subghz_environment_free(env);
            furi_string_free(proto_str);
            furi_string_free(preset_str);
            if(custom_data) free(custom_data);
            break;
        }

        subghz_devices_start_async_tx(dev, subghz_transmitter_yield, tx);

        /* Wait up to 30 s for the async TX to finish */
        uint32_t timeout = 30000u;
        while(!subghz_devices_is_async_complete_tx(dev) && timeout > 0u) {
            furi_delay_ms(10);
            timeout -= 10u;
        }

        subghz_devices_stop_async_tx(dev);
        subghz_devices_idle(dev);

        subghz_transmitter_free(tx);
        subghz_environment_free(env);
        furi_string_free(proto_str);
        furi_string_free(preset_str);
        if(custom_data) free(custom_data);

        ok = true;
    } while(false);

    flipper_format_free(fff);
    furi_record_close(RECORD_STORAGE);
    return ok;
}

/* TX worker thread — keeps the UI responsive during transmission */
static int32_t tx_thread_fn(void* ctx) {
    GpsGarageApp* app = ctx;
    subghz_transmit_file(app->config.subghz_file);
    app->is_transmitting = false;
    view_dispatcher_send_custom_event(app->view_dispatcher, APP_EVT_TX_DONE);
    return 0;
}

static void start_tx(GpsGarageApp* app) {
    if(app->is_transmitting) return;
    if(app->config.subghz_file[0] == '\0') return;

    app->is_transmitting = true;
    app->last_tx_tick    = furi_get_tick();

    if(app->tx_thread) {
        furi_thread_join(app->tx_thread);
        furi_thread_free(app->tx_thread);
    }
    app->tx_thread =
        furi_thread_alloc_ex("SubGhzTX", 2 * 1024, tx_thread_fn, app);
    furi_thread_start(app->tx_thread);
}

/* ══════════════════════════════════════════════════════════════════════
 * Draw callback
 * ══════════════════════════════════════════════════════════════════════ */

static void main_view_draw(Canvas* canvas, void* model) {
    MainViewModel* vm  = model;
    GpsGarageApp*  app = vm->app;

    /* Snapshot GPS data under mutex to avoid tearing */
    furi_mutex_acquire(app->gps_mutex, FuriWaitForever);
    bool    fix  = app->gps_fix;
    uint8_t sats = app->gps_satellites;
    double  clat = app->gps_lat;
    double  clon = app->gps_lon;
    furi_mutex_release(app->gps_mutex);

    canvas_clear(canvas);

    /* ── Title bar ─────────────────────────────────────────────────── */
    canvas_set_font(canvas, FontPrimary);
    canvas_draw_str(canvas, 0, 10, "GPS Garage Opener");
    canvas_set_font(canvas, FontSecondary);

    char sat_buf[10];
    if(fix)
        snprintf(sat_buf, sizeof(sat_buf), "[%us]", (unsigned)sats);
    else
        snprintf(sat_buf, sizeof(sat_buf), "[NoFix]");
    canvas_draw_str(canvas, 128 - (int)(strlen(sat_buf) * 6), 10, sat_buf);

    canvas_draw_line(canvas, 0, 12, 127, 12);

    /* ── Current GPS position ──────────────────────────────────────── */
    char row[32];
    if(fix)
        snprintf(row, sizeof(row), "%.5f  %.5f", clat, clon);
    else
        snprintf(row, sizeof(row), "Searching for fix...");
    canvas_draw_str(canvas, 0, 22, "Pos:");
    canvas_draw_str(canvas, 24, 22, row);

    /* ── Target position ───────────────────────────────────────────── */
    if(app->config.target_set)
        snprintf(
            row, sizeof(row), "%.5f  %.5f",
            app->config.target_lat, app->config.target_lon);
    else
        snprintf(row, sizeof(row), "Not set (hold UP)");
    canvas_draw_str(canvas, 0, 32, "Tgt:");
    canvas_draw_str(canvas, 24, 32, row);

    /* ── Distance + radius ─────────────────────────────────────────── */
    if(app->config.target_set && fix) {
        if(app->current_distance_m < 1000.0f)
            snprintf(
                row, sizeof(row), "Dist:%.0fm  R:%.0fm",
                (double)app->current_distance_m,
                (double)app->config.radius_m);
        else
            snprintf(
                row, sizeof(row), "Dist:%.1fkm  R:%.0fm",
                (double)(app->current_distance_m / 1000.0f),
                (double)app->config.radius_m);
    } else {
        snprintf(row, sizeof(row), "Radius: %.0fm  (</> adj)", (double)app->config.radius_m);
    }
    canvas_draw_str(canvas, 0, 42, row);

    /* ── SubGHz file ───────────────────────────────────────────────── */
    const char* fname = NULL;
    if(app->config.subghz_file[0]) {
        fname = strrchr(app->config.subghz_file, '/');
        fname = fname ? fname + 1 : app->config.subghz_file;
    }
    canvas_draw_str(canvas, 0, 52, "Sub:");
    canvas_draw_str(canvas, 24, 52, fname ? fname : "(none - hold DN)");

    /* ── Status line ───────────────────────────────────────────────── */
    const char* status;
    if(app->is_transmitting)
        status = "TRANSMITTING...";
    else if(app->in_range && app->config.tracking_enabled)
        status = "IN RANGE - will TX";
    else if(app->config.tracking_enabled)
        status = "TRACKING  (holdOK off)";
    else
        status = "IDLE  (holdOK = track)";
    canvas_draw_str(canvas, 0, 63, status);
}

/* ══════════════════════════════════════════════════════════════════════
 * Input callback
 * ══════════════════════════════════════════════════════════════════════ */

static bool main_view_input(InputEvent* event, void* ctx) {
    GpsGarageApp* app = ctx;

    if(event->type == InputTypeShort) {
        switch(event->key) {
        case InputKeyOk:
            start_tx(app);
            return true;
        case InputKeyLeft:
            app->config.radius_m -= CONFIG_RADIUS_STEP_M;
            if(app->config.radius_m < CONFIG_RADIUS_MIN_M)
                app->config.radius_m = CONFIG_RADIUS_MIN_M;
            config_save(&app->config);
            with_view_model(app->main_view, MainViewModel * vm, { (void)vm; }, true);
            return true;
        case InputKeyRight:
            app->config.radius_m += CONFIG_RADIUS_STEP_M;
            if(app->config.radius_m > CONFIG_RADIUS_MAX_M)
                app->config.radius_m = CONFIG_RADIUS_MAX_M;
            config_save(&app->config);
            with_view_model(app->main_view, MainViewModel * vm, { (void)vm; }, true);
            return true;
        default:
            break;
        }
    }

    if(event->type == InputTypeLong) {
        switch(event->key) {
        case InputKeyOk:
            app->config.tracking_enabled = !app->config.tracking_enabled;
            config_save(&app->config);
            with_view_model(app->main_view, MainViewModel * vm, { (void)vm; }, true);
            return true;

        case InputKeyUp: {
            /* Capture current GPS position as trigger target */
            furi_mutex_acquire(app->gps_mutex, FuriWaitForever);
            bool got_fix = app->gps_fix;
            if(got_fix) {
                app->config.target_lat = app->gps_lat;
                app->config.target_lon = app->gps_lon;
                app->config.target_set = true;
            }
            furi_mutex_release(app->gps_mutex);
            if(got_fix) {
                config_save(&app->config);
                with_view_model(
                    app->main_view, MainViewModel * vm, { (void)vm; }, true);
            }
            return true;
        }

        case InputKeyDown: {
            /* Browse SD card for a .sub file */
            FuriString* path = furi_string_alloc_set(EXT_PATH("subghz"));
            DialogsFileBrowserOptions opts;
            dialog_file_browser_set_basic_options(&opts, ".sub", NULL);
            opts.base_path      = EXT_PATH("subghz");
            opts.skip_assets    = true;
            opts.hide_dot_files = true;

            if(dialog_file_browser_show(app->dialogs, path, path, &opts)) {
                strncpy(
                    app->config.subghz_file,
                    furi_string_get_cstr(path),
                    CONFIG_SUB_PATH_MAX - 1);
                app->config.subghz_file[CONFIG_SUB_PATH_MAX - 1] = '\0';
                config_save(&app->config);
                with_view_model(
                    app->main_view, MainViewModel * vm, { (void)vm; }, true);
            }
            furi_string_free(path);
            return true;
        }

        default:
            break;
        }
    }

    return false; /* pass BACK to the navigation callback */
}

/* ══════════════════════════════════════════════════════════════════════
 * ViewDispatcher callbacks
 * ══════════════════════════════════════════════════════════════════════ */

static bool navigation_callback(void* ctx) {
    GpsGarageApp* app = ctx;
    view_dispatcher_stop(app->view_dispatcher);
    return true;
}

static bool custom_event_callback(void* ctx, uint32_t event) {
    GpsGarageApp* app = ctx;

    if(event == APP_EVT_GPS_UPDATE || event == APP_EVT_TX_DONE) {
        /* Recompute distance on any GPS update or after TX completes */
        if(app->config.target_set) {
            furi_mutex_acquire(app->gps_mutex, FuriWaitForever);
            bool   fix  = app->gps_fix;
            double clat = app->gps_lat;
            double clon = app->gps_lon;
            furi_mutex_release(app->gps_mutex);

            if(fix) {
                app->current_distance_m = geo_distance_m(
                    clat, clon,
                    app->config.target_lat,
                    app->config.target_lon);
                app->in_range =
                    (app->current_distance_m <= app->config.radius_m);
            }
        }
        with_view_model(app->main_view, MainViewModel * vm, { (void)vm; }, true);
        return true;
    }

    if(event == APP_EVT_TIMER_TICK) {
        if(app->config.tracking_enabled &&
           app->config.target_set &&
           app->config.subghz_file[0] != '\0' &&
           !app->is_transmitting) {

            furi_mutex_acquire(app->gps_mutex, FuriWaitForever);
            bool   fix  = app->gps_fix;
            double clat = app->gps_lat;
            double clon = app->gps_lon;
            furi_mutex_release(app->gps_mutex);

            if(fix) {
                float dist = geo_distance_m(
                    clat, clon,
                    app->config.target_lat,
                    app->config.target_lon);
                app->current_distance_m = dist;
                app->in_range           = (dist <= app->config.radius_m);

                if(app->in_range) {
                    uint32_t elapsed_ms  = furi_get_tick() - app->last_tx_tick;
                    uint32_t cooldown_ms = app->config.cooldown_s * 1000u;
                    if(elapsed_ms >= cooldown_ms) {
                        start_tx(app);
                    }
                }
            }
        }
        with_view_model(app->main_view, MainViewModel * vm, { (void)vm; }, true);
        return true;
    }

    return false;
}

/* Tick fires every 1 s on the ViewDispatcher thread */
static void tick_callback(void* ctx) {
    GpsGarageApp* app = ctx;
    view_dispatcher_send_custom_event(app->view_dispatcher, APP_EVT_TIMER_TICK);
}

/* ══════════════════════════════════════════════════════════════════════
 * Application entry point
 * ══════════════════════════════════════════════════════════════════════ */

int32_t gps_garage_app(void* p) {
    UNUSED(p);

    GpsGarageApp* app = malloc(sizeof(GpsGarageApp));
    memset(app, 0, sizeof(*app));

    config_load(&app->config);
    /*
     * Allow an immediate first transmission on the first entry into range
     * by pretending the last TX happened just over one cooldown period ago.
     */
    app->last_tx_tick =
        furi_get_tick() - (app->config.cooldown_s + 1u) * 1000u;

    app->gps_mutex = furi_mutex_alloc(FuriMutexTypeNormal);

    /* One-time SubGHz device list init */
    subghz_devices_init();

    /* Open services */
    app->dialogs = furi_record_open(RECORD_DIALOGS);
    app->gui     = furi_record_open(RECORD_GUI);

    /* ── Build main view ──────────────────────────────────────────── */
    app->main_view = view_alloc();
    view_set_context(app->main_view, app);
    view_set_draw_callback(app->main_view, main_view_draw);
    view_set_input_callback(app->main_view, main_view_input);
    view_allocate_model(
        app->main_view, ViewModelTypeLockFree, sizeof(MainViewModel));
    with_view_model(
        app->main_view,
        MainViewModel * vm,
        { vm->app = app; },
        false);

    /* ── ViewDispatcher ───────────────────────────────────────────── */
    app->view_dispatcher = view_dispatcher_alloc();
    view_dispatcher_set_event_callback_context(app->view_dispatcher, app);
    view_dispatcher_set_navigation_event_callback(
        app->view_dispatcher, navigation_callback);
    view_dispatcher_set_custom_event_callback(
        app->view_dispatcher, custom_event_callback);
    view_dispatcher_set_tick_event_callback(
        app->view_dispatcher, tick_callback, 1000);

    view_dispatcher_attach_to_gui(
        app->view_dispatcher, app->gui, ViewDispatcherTypeFullscreen);
    view_dispatcher_add_view(app->view_dispatcher, 0, app->main_view);
    view_dispatcher_switch_to_view(app->view_dispatcher, 0);

    /* ── GPS worker ───────────────────────────────────────────────── */
    app->gps_worker = gps_worker_alloc(app);
    gps_worker_start(app->gps_worker);

    /* ── Run (blocks until BACK pressed) ─────────────────────────── */
    view_dispatcher_run(app->view_dispatcher);

    /* ══ Teardown ════════════════════════════════════════════════ */

    gps_worker_stop(app->gps_worker);
    gps_worker_free(app->gps_worker);

    if(app->tx_thread) {
        furi_thread_join(app->tx_thread);
        furi_thread_free(app->tx_thread);
    }

    subghz_devices_deinit();

    view_dispatcher_remove_view(app->view_dispatcher, 0);
    view_free(app->main_view);
    view_dispatcher_free(app->view_dispatcher);

    furi_record_close(RECORD_GUI);
    furi_record_close(RECORD_DIALOGS);

    furi_mutex_free(app->gps_mutex);
    free(app);
    return 0;
}
