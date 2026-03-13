#pragma once

#include <furi.h>
#include <gui/gui.h>
#include <gui/view_dispatcher.h>
#include <dialogs/dialogs.h>
#include <notification/notification.h>
#include "config.h"

/* Forward declarations to avoid circular includes */
struct GpsWorker; /* defined in gps.h */

/* Custom events sent to the ViewDispatcher event loop */
typedef enum {
    APP_EVT_GPS_UPDATE  = 0, /* GPS worker updated position     */
    APP_EVT_TIMER_TICK  = 1, /* Periodic position-check tick    */
    APP_EVT_TX_DONE     = 2, /* SubGHz TX thread finished       */
} AppCustomEvent;

typedef struct GpsGarageApp {
    /* ── UI ──────────────────────────────────────────────────────── */
    Gui*              gui;
    ViewDispatcher*   view_dispatcher;
    View*             main_view;
    DialogsApp*       dialogs;
    NotificationApp*  notifications;

    /* ── GPS state (protected by gps_mutex) ───────────────────────── */
    FuriMutex* gps_mutex;
    double     gps_lat;
    double     gps_lon;
    bool       gps_fix;
    uint8_t    gps_satellites;

    /* ── Runtime state ────────────────────────────────────────────── */
    float    current_distance_m; /* last computed distance to target */
    bool     in_range;
    bool     trigger_armed;      /* true = will fire on next zone entry */
    bool     is_transmitting;

    /* ── Exit lock (requires 3× BACK while tracking) ──────────────── */
    uint8_t  back_press_count;
    uint32_t back_press_tick;    /* furi_get_tick() of first press in sequence */

    /* ── Sub-components ───────────────────────────────────────────── */
    struct GpsWorker* gps_worker;
    FuriTimer*        position_timer;
    FuriThread*       tx_thread;

    /* ── Persistent config ────────────────────────────────────────── */
    AppConfig config;
} GpsGarageApp;

/* Entry point declared for application.fam */
int32_t gps_garage_app(void* p);
