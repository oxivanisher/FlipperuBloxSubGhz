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
#include <notification/notification.h>
#include <notification/notification_messages.h>
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

/* Exit-lock constants (3× BACK within 2 s required while tracking) */
#define BACK_LOCK_PRESSES   3u
#define BACK_LOCK_WINDOW_MS 2000u

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

/*
 * Transmit the .sub file `count` times with `delay_ms` between each send.
 * The CC1101 is initialised once for the whole burst to avoid re-init races.
 */
static bool subghz_transmit_burst(const char* filepath, uint8_t count, uint32_t delay_ms) {
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
        furi_string_free(preset_str);

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

        /* Ensure a reliable repeat count per send (matches stock SubGHz app behaviour) */
        uint32_t repeat = 5u;
        flipper_format_insert_or_update_uint32(fff, "Repeat", &repeat, 1);

        /* Initialise the CC1101 ONCE for the whole burst */
        const SubGhzDevice* dev =
            subghz_devices_get_by_name(SUBGHZ_DEVICE_CC1101_INT_NAME);
        subghz_devices_reset(dev);
        subghz_devices_idle(dev);
        subghz_devices_load_preset(dev, preset_enum, custom_data);
        subghz_devices_set_frequency(dev, frequency);

        if(!subghz_devices_set_tx(dev)) {
            /* Frequency blocked by region settings */
            subghz_devices_idle(dev);
            furi_string_free(proto_str);
            if(custom_data) free(custom_data);
            break;
        }

        ok = true;

        /* Burst loop — only transmitter state is reset between sends.
         * subghz_transmitter_deserialize rewinds the format internally. */
        for(uint8_t i = 0; i < count; i++) {
            SubGhzEnvironment* env = subghz_environment_alloc();
            subghz_environment_set_protocol_registry(env, &subghz_protocol_registry);
            SubGhzTransmitter* tx =
                subghz_transmitter_alloc_init(env, furi_string_get_cstr(proto_str));

            if(!tx) {
                subghz_environment_free(env);
                ok = false;
                break;
            }

            if(subghz_transmitter_deserialize(tx, fff) != SubGhzProtocolStatusOk) {
                subghz_transmitter_free(tx);
                subghz_environment_free(env);
                ok = false;
                break;
            }

            subghz_devices_start_async_tx(dev, subghz_transmitter_yield, tx);

            uint32_t timeout = 30000u;
            while(!subghz_devices_is_async_complete_tx(dev) && timeout > 0u) {
                furi_delay_ms(10);
                timeout -= 10u;
            }
            subghz_devices_stop_async_tx(dev);

            subghz_transmitter_free(tx);
            subghz_environment_free(env);

            if(i < count - 1u) furi_delay_ms(delay_ms);
        }

        subghz_devices_idle(dev);
        furi_string_free(proto_str);
        if(custom_data) free(custom_data);

    } while(false);

    flipper_format_free(fff);
    furi_record_close(RECORD_STORAGE);
    return ok;
}

/* TX worker thread — keeps the UI responsive during transmission */
/* 3 sends per second for 5 seconds = 15 total */
#define TX_BURST_COUNT    15u
#define TX_BURST_DELAY_MS 333u

static int32_t tx_thread_fn(void* ctx) {
    GpsGarageApp* app = ctx;
    notification_message(app->notifications, &sequence_blink_start_blue);
    subghz_transmit_burst(app->config.subghz_file, TX_BURST_COUNT, TX_BURST_DELAY_MS);
    notification_message(app->notifications, &sequence_blink_stop);
    app->is_transmitting = false;
    view_dispatcher_send_custom_event(app->view_dispatcher, APP_EVT_TX_DONE);
    return 0;
}

static void start_tx(GpsGarageApp* app) {
    if(app->is_transmitting) return;
    if(app->config.subghz_file[0] == '\0') return;

    app->is_transmitting = true;

    if(app->tx_thread) {
        furi_thread_join(app->tx_thread);
        furi_thread_free(app->tx_thread);
    }
    app->tx_thread =
        furi_thread_alloc_ex("SubGhzTX", 4 * 1024, tx_thread_fn, app);
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
    canvas_draw_str(canvas, 0, 10, "GPS Garage");
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
    char status_buf[24];
    const char* status;
    if(app->back_press_count > 0) {
        snprintf(
            status_buf, sizeof(status_buf),
            "BACK x%u to stop",
            (unsigned)(BACK_LOCK_PRESSES - app->back_press_count));
        status = status_buf;
    } else if(app->is_transmitting) {
        status = "TRANSMITTING...";
    } else if(app->in_range && app->config.tracking_enabled && app->trigger_armed) {
        status = "IN RANGE - will TX";
    } else if(app->in_range && app->config.tracking_enabled) {
        status = "IN RANGE - sent";
    } else if(app->config.tracking_enabled) {
        status = "TRACKING (3x back)";
    } else {
        status = "IDLE (OK=track)";
    }
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
            if(!app->config.tracking_enabled) {
                app->config.tracking_enabled = true;
                config_save(&app->config);
                with_view_model(app->main_view, MainViewModel * vm, { (void)vm; }, true);
            } else {
                start_tx(app);
            }
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

    /* No lock when tracking is off */
    if(!app->config.tracking_enabled) {
        view_dispatcher_stop(app->view_dispatcher);
        return true;
    }

    uint32_t now = furi_get_tick();

    /* Reset the counter if the window expired */
    if(app->back_press_count > 0 &&
       (now - app->back_press_tick) > BACK_LOCK_WINDOW_MS) {
        app->back_press_count = 0;
    }

    if(app->back_press_count == 0) {
        app->back_press_tick = now;
    }
    app->back_press_count++;

    if(app->back_press_count >= BACK_LOCK_PRESSES) {
        app->back_press_count = 0;
        app->config.tracking_enabled = false;
        config_save(&app->config);
        with_view_model(app->main_view, MainViewModel * vm, { (void)vm; }, true);
    } else {
        /* Redraw so the "BACK xN to stop" prompt updates immediately */
        with_view_model(app->main_view, MainViewModel * vm, { (void)vm; }, true);
    }
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
                bool now_in_range = (dist <= app->config.radius_m);

                if(!now_in_range) {
                    /* Outside zone: re-arm so next entry fires */
                    app->trigger_armed = true;
                } else if(app->trigger_armed) {
                    /* Just entered (or started inside) the zone */
                    app->trigger_armed = false;
                    start_tx(app);
                }

                app->in_range = now_in_range;
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

    /* Auto-enable tracking if everything is already configured */
    if(app->config.target_set && app->config.subghz_file[0] != '\0') {
        app->config.tracking_enabled = true;
    }

    /* Armed: will fire the moment the zone is first entered */
    app->trigger_armed = true;

    app->gps_mutex = furi_mutex_alloc(FuriMutexTypeNormal);

    /* One-time SubGHz device list init */
    subghz_devices_init();

    /* Open services */
    app->notifications = furi_record_open(RECORD_NOTIFICATION);
    app->dialogs       = furi_record_open(RECORD_DIALOGS);
    app->gui           = furi_record_open(RECORD_GUI);

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

    /* Reset LED in case we exited mid-burst */
    notification_message(app->notifications, &sequence_reset_rgb);

    view_dispatcher_remove_view(app->view_dispatcher, 0);
    view_free(app->main_view);
    view_dispatcher_free(app->view_dispatcher);

    furi_record_close(RECORD_GUI);
    furi_record_close(RECORD_DIALOGS);
    furi_record_close(RECORD_NOTIFICATION);

    furi_mutex_free(app->gps_mutex);
    free(app);
    return 0;
}
