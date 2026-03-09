#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    double   latitude;    /* decimal degrees, positive = North */
    double   longitude;   /* decimal degrees, positive = East  */
    bool     valid;       /* A = Active fix */
    uint8_t  satellites;  /* number of satellites in use (from GGA) */
    float    speed_knots; /* speed over ground */
} NmeaData;

/**
 * Parse a $GPRMC / $GNRMC sentence.
 * @param sentence  Null-terminated NMEA sentence starting with '$'.
 * @param data      Output structure; partially updated even on void fix.
 * @return true if the sentence type matched (regardless of fix validity).
 */
bool nmea_parse_rmc(const char* sentence, NmeaData* data);

/**
 * Parse a $GPGGA / $GNGGA sentence for satellite count.
 * @return true if the sentence type matched.
 */
bool nmea_parse_gga(const char* sentence, NmeaData* data);
