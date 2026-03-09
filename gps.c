#include "gps.h"
#include "app.h" /* GpsGarageApp definition */

#include <furi.h>
#include <furi_hal_serial.h>
#include <furi_hal_serial_control.h>
#include <gui/view_dispatcher.h>
#include <string.h>

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

    /* Acquire and configure serial port */
    worker->serial = furi_hal_serial_control_acquire(FuriHalSerialIdUsart);
    furi_assert(worker->serial);
    furi_hal_serial_init(worker->serial, GPS_BAUD_RATE);
    furi_hal_serial_async_rx_start(worker->serial, uart_rx_cb, worker, false);

    /* NMEA accumulation buffer */
    char    nmea[128];
    size_t  nmea_pos = 0;
    NmeaData parsed  = {0};

    while(true) {
        uint32_t flags = furi_thread_flags_wait(
            EVT_STOP | EVT_RXDATA, FuriFlagWaitAny, FuriWaitForever);

        if(flags & FuriFlagError) continue;
        if(flags & EVT_STOP) break;

        /* Drain the stream buffer byte by byte and build NMEA sentences */
        uint8_t byte;
        while(furi_stream_buffer_receive(worker->rx_stream, &byte, 1, 0) == 1) {
            if(byte == '$') {
                nmea_pos    = 0;
                nmea[nmea_pos++] = (char)byte;
            } else if(nmea_pos > 0) {
                if(nmea_pos < sizeof(nmea) - 1) {
                    nmea[nmea_pos++] = (char)byte;
                }
                if(byte == '\n') {
                    nmea[nmea_pos] = '\0';

                    if(nmea_parse_rmc(nmea, &parsed)) {
                        furi_mutex_acquire(app->gps_mutex, FuriWaitForever);
                        app->gps_fix = parsed.valid;
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
    w->thread    = furi_thread_alloc_ex("GpsWorker", 1024, gps_worker_thread, w);
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
