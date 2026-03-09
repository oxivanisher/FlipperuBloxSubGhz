#pragma once

#include <furi.h>
#include <furi_hal_serial.h>
#include <furi_hal_serial_control.h>
#include "nmea_parser.h"

#define GPS_BAUD_RATE 9600u
#define GPS_RX_BUF    2048u

/* Forward-declare the app struct tag only — no typedef to avoid redefinition
   when app.h (which provides the full typedef) is included in the same TU. */
struct GpsGarageApp;

/* Named struct tag so app.h can forward-declare 'struct GpsWorker' */
typedef struct GpsWorker {
    struct GpsGarageApp* app;
    FuriHalSerialHandle* serial;
    FuriStreamBuffer*    rx_stream;
    FuriThread*          thread;
} GpsWorker;

/* NOTE: call sites that already #include "app.h" may pass GpsGarageApp*
   directly — struct GpsGarageApp* and GpsGarageApp* are the same type. */
GpsWorker* gps_worker_alloc(struct GpsGarageApp* app);
void       gps_worker_free(GpsWorker* worker);

/* Start / stop the background UART-reader thread */
void gps_worker_start(GpsWorker* worker);
void gps_worker_stop(GpsWorker* worker);
