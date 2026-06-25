#include "gps.h"
#include "app.h" /* GpsGarageApp definition */

#include <furi.h>
#include <furi_hal_serial.h>
#include <furi_hal_serial_control.h>
#include <gui/view_dispatcher.h>
#include <string.h>

/* ── UBX position hint ────────────────────────────────────────────────────
 * Sends the approximate position to the module before NMEA starts.
 * Converts a cold start (full sky search) into a warm start (~5–10 s TTFF).
 * We send both AID-INI (M5–M9) and MGA-INI-POS_LLH (M9+/M10); the module
 * silently ignores whichever message it does not understand.
 * Accuracy is declared as 20 km so the module treats it as a rough hint only.
 */
static void gps_send_position_hint(FuriHalSerialHandle* serial, double lat, double lon) {
    /* lat/lon in 1e-7 degrees, integer arithmetic to avoid float promotion */
    int32_t  lat7   = (int32_t)(lat * 10000000);
    int32_t  lon7   = (int32_t)(lon * 10000000);
    uint32_t acc_cm = 2000000u;  /* 20 km in cm — conservative for "a few km" use case */
    uint32_t acc_mm = 20000000u; /* 20 km in mm — same value for MGA message           */

    /* ── UBX-AID-INI (class 0x0B, id 0x01, payload 48 B) ── */
    {
        uint8_t payload[48];
        memset(payload, 0, sizeof(payload));
        memcpy(payload +  0, &lat7,   4); /* X / Lat  */
        memcpy(payload +  4, &lon7,   4); /* Y / Lon  */
        /* altitude (payload+8) left at 0 (sea level guess)  */
        memcpy(payload + 12, &acc_cm, 4); /* posAcc   */
        uint32_t flags = 0x00000021u;     /* bit0=posValid, bit5=lla */
        memcpy(payload + 36, &flags,  4);

        uint8_t frame[56];
        frame[0] = 0xB5; frame[1] = 0x62;
        frame[2] = 0x0B; frame[3] = 0x01; /* AID-INI */
        frame[4] = 48;   frame[5] = 0;
        memcpy(frame + 6, payload, 48);

        uint8_t ck_a = 0, ck_b = 0;
        for(size_t i = 2; i < 54; i++) { ck_a += frame[i]; ck_b += ck_a; }
        frame[54] = ck_a;
        frame[55] = ck_b;
        furi_hal_serial_tx(serial, frame, 56);
    }

    /* ── UBX-MGA-INI-POS_LLH (class 0x13, id 0x40, payload 20 B) ── */
    {
        uint8_t payload[20];
        memset(payload, 0, sizeof(payload));
        payload[0] = 0x01; /* type = LLH */
        /* version (payload[1]) = 0 */
        memcpy(payload +  4, &lat7,  4);
        memcpy(payload +  8, &lon7,  4);
        /* altitude (payload+12) left at 0                   */
        memcpy(payload + 16, &acc_mm, 4);

        uint8_t frame[28];
        frame[0] = 0xB5; frame[1] = 0x62;
        frame[2] = 0x13; frame[3] = 0x40; /* MGA-INI */
        frame[4] = 20;   frame[5] = 0;
        memcpy(frame + 6, payload, 20);

        uint8_t ck_a = 0, ck_b = 0;
        for(size_t i = 2; i < 26; i++) { ck_a += frame[i]; ck_b += ck_a; }
        frame[26] = ck_a;
        frame[27] = ck_b;
        furi_hal_serial_tx(serial, frame, 28);
    }

    furi_delay_ms(100); /* allow module to process before NMEA RX starts */
}

/* ── thread-flag bits ─────────────────────────────────────────────────── */
#define EVT_STOP   (1u << 0)
#define EVT_RXDATA (1u << 1)

/* ── UART IRQ callback (interrupt context – keep it minimal) ──────────── */
static void uart_rx_cb(
    FuriHalSerialHandle* handle,
    FuriHalSerialRxEvent event,
    void* ctx) {
    GpsWorker* worker = ctx;
    if(event & FuriHalSerialRxEventData) {
        uint8_t byte = furi_hal_serial_async_rx(handle);
        furi_stream_buffer_send(worker->rx_stream, &byte, 1, 0);
        furi_thread_flags_set(furi_thread_get_id(worker->thread), EVT_RXDATA);
    }
}

/* ── worker thread ────────────────────────────────────────────────────── */
static int32_t gps_worker_thread(void* ctx) {
    GpsWorker*    worker = ctx;
    GpsGarageApp* app    = worker->app;

    /* Acquire and configure serial port.
     * When launched via shortcut the debug console may still hold the UART;
     * retry a few times with a short delay before giving up gracefully. */
    for(uint8_t tries = 0; tries < 10 && !worker->serial; tries++) {
        worker->serial = furi_hal_serial_control_acquire(FuriHalSerialIdUsart);
        if(!worker->serial) furi_delay_ms(50);
    }
    if(!worker->serial) {
        /* Unable to get the serial port — run without GPS data */
        furi_thread_flags_wait(EVT_STOP, FuriFlagWaitAny, FuriWaitForever);
        return 0;
    }
    furi_hal_serial_init(worker->serial, GPS_BAUD_RATE);
    furi_hal_serial_async_rx_start(worker->serial, uart_rx_cb, worker, false);

    /* Warm-start hint: tell the module roughly where it is.
     * Sent after async_rx_start so the UART HAL is fully armed before TX. */
    if(app->config.target_set) {
        gps_send_position_hint(
            worker->serial,
            app->config.target_lat,
            app->config.target_lon);
    }

    /* NMEA accumulation buffer */
    char     nmea[128];
    size_t   nmea_pos      = 0;
    NmeaData parsed        = {0};
    uint32_t last_data_tick = 0; /* furi_get_tick() of last received byte; 0 = never */

#define GPS_MODULE_TIMEOUT_MS 3000u

    while(true) {
        uint32_t flags = furi_thread_flags_wait(
            EVT_STOP | EVT_RXDATA, FuriFlagWaitAny, GPS_MODULE_TIMEOUT_MS);

        if(flags & FuriFlagError) {
            /* Timeout (or spurious error). If no data has arrived recently,
             * the GPS module is absent or not yet powered. Clear the state so
             * the UI can show "No GPS module" instead of "Searching for fix". */
            uint32_t now = furi_get_tick();
            if(last_data_tick == 0 || (now - last_data_tick) >= GPS_MODULE_TIMEOUT_MS) {
                furi_mutex_acquire(app->gps_mutex, FuriWaitForever);
                bool was_present = app->gps_module_present;
                app->gps_module_present = false;
                app->gps_fix            = false;
                app->gps_satellites     = 0;
                furi_mutex_release(app->gps_mutex);
                if(was_present) {
                    view_dispatcher_send_custom_event(
                        app->view_dispatcher, APP_EVT_GPS_UPDATE);
                }
            }
            continue;
        }
        if(flags & EVT_STOP) break;

        last_data_tick = furi_get_tick();

        /* Drain the stream buffer byte by byte and build NMEA sentences */
        uint8_t byte;
        while(furi_stream_buffer_receive(worker->rx_stream, &byte, 1, 0) == 1) {
            if(byte == '$') {
                nmea_pos         = 0;
                nmea[nmea_pos++] = (char)byte;
            } else if(nmea_pos > 0) {
                if(nmea_pos < sizeof(nmea) - 1) {
                    nmea[nmea_pos++] = (char)byte;
                }
                if(byte == '\n') {
                    nmea[nmea_pos] = '\0';

                    if(nmea_parse_rmc(nmea, &parsed)) {
                        furi_mutex_acquire(app->gps_mutex, FuriWaitForever);
                        app->gps_module_present = true;
                        app->gps_fix            = parsed.valid;
                        if(parsed.valid) {
                            app->gps_lat = parsed.latitude;
                            app->gps_lon = parsed.longitude;
                        }
                        furi_mutex_release(app->gps_mutex);
                        view_dispatcher_send_custom_event(
                            app->view_dispatcher, APP_EVT_GPS_UPDATE);
                    } else if(nmea_parse_gga(nmea, &parsed)) {
                        furi_mutex_acquire(app->gps_mutex, FuriWaitForever);
                        app->gps_satellites = parsed.satellites;
                        furi_mutex_release(app->gps_mutex);
                    }

                    nmea_pos = 0;
                }
            }
        }
    }

    furi_hal_serial_async_rx_stop(worker->serial);
    furi_hal_serial_deinit(worker->serial);
    furi_hal_serial_control_release(worker->serial);
    worker->serial = NULL;
    return 0;
}

/* ── public API ───────────────────────────────────────────────────────── */

GpsWorker* gps_worker_alloc(GpsGarageApp* app) {
    GpsWorker* w = malloc(sizeof(GpsWorker));
    w->app       = app;
    w->serial    = NULL;
    w->rx_stream = furi_stream_buffer_alloc(GPS_RX_BUF, 1);
    w->thread    = furi_thread_alloc_ex("GpsWorker", 2048, gps_worker_thread, w);
    return w;
}

void gps_worker_free(GpsWorker* worker) {
    furi_thread_free(worker->thread);
    furi_stream_buffer_free(worker->rx_stream);
    free(worker);
}

void gps_worker_start(GpsWorker* worker) {
    furi_thread_start(worker->thread);
}

void gps_worker_stop(GpsWorker* worker) {
    furi_thread_flags_set(furi_thread_get_id(worker->thread), EVT_STOP);
    furi_thread_join(worker->thread);
}
