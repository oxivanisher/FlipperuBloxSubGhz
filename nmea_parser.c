#include "nmea_parser.h"

#include <string.h>
#include <stdlib.h>

/* ── helpers ──────────────────────────────────────────────────────────── */

/* Copy the Nth comma-delimited field (0-based) of sentence into buf.
   Returns buf on success, NULL if the field does not exist. */
static const char* field(const char* sentence, int n, char* buf, size_t len) {
    int f = 0;
    const char* p = sentence;

    while(*p && f < n) {
        if(*p++ == ',') f++;
    }
    if(!*p || f != n) {
        buf[0] = '\0';
        return NULL;
    }
    size_t i = 0;
    while(*p && *p != ',' && *p != '*' && i < len - 1) {
        buf[i++] = *p++;
    }
    buf[i] = '\0';
    return buf;
}

/* Convert NMEA DDMM.MMMMM notation to decimal degrees.
   Use integer literals (100, 60) to avoid -Werror=double-promotion. */
static double to_decimal(const char* coord, char dir) {
    if(!coord || !*coord) return 0;
    double v   = strtod(coord, NULL);
    int    deg = (int)(v / 100);
    double min = v - (double)(deg * 100);
    double dec = (double)deg + min / 60;
    if(dir == 'S' || dir == 'W') dec = -dec;
    return dec;
}

/* Verify the XOR checksum following the '*' in an NMEA sentence. */
static bool checksum_ok(const char* sentence) {
    if(!sentence || *sentence != '$') return false;
    const char* p = sentence + 1;
    uint8_t     cs = 0;
    while(*p && *p != '*') cs ^= (uint8_t)*p++;
    if(*p != '*') return false;
    p++;
    /* Parse two-hex-digit expected checksum */
    uint8_t hi = (uint8_t)(p[0] >= 'A' ? (p[0] - 'A' + 10) : (p[0] - '0'));
    uint8_t lo = (uint8_t)(p[1] >= 'A' ? (p[1] - 'A' + 10) : (p[1] - '0'));
    return cs == (uint8_t)((hi << 4) | lo);
}

/* ── public API ───────────────────────────────────────────────────────── */

bool nmea_parse_rmc(const char* sentence, NmeaData* data) {
    if(!sentence || !data) return false;
    if(strncmp(sentence, "$GPRMC", 6) != 0 &&
       strncmp(sentence, "$GNRMC", 6) != 0) return false;
    if(!checksum_ok(sentence)) return false;

    char buf[24];

    /* Field 2: Status */
    if(!field(sentence, 2, buf, sizeof(buf))) return false;
    if(buf[0] != 'A') {
        data->valid = false;
        return true; /* Sentence recognised, just no fix */
    }

    char lat_s[16], lat_d[4], lon_s[16], lon_d[4];
    if(!field(sentence, 3, lat_s, sizeof(lat_s))) return false;
    if(!field(sentence, 4, lat_d, sizeof(lat_d))) return false;
    if(!field(sentence, 5, lon_s, sizeof(lon_s))) return false;
    if(!field(sentence, 6, lon_d, sizeof(lon_d))) return false;

    data->latitude  = to_decimal(lat_s, lat_d[0]);
    data->longitude = to_decimal(lon_s, lon_d[0]);

    field(sentence, 7, buf, sizeof(buf));
    data->speed_knots = buf[0] ? (float)strtod(buf, NULL) : 0.0f;

    data->valid = true;
    return true;
}

bool nmea_parse_gga(const char* sentence, NmeaData* data) {
    if(!sentence || !data) return false;
    if(strncmp(sentence, "$GPGGA", 6) != 0 &&
       strncmp(sentence, "$GNGGA", 6) != 0) return false;

    char buf[8];
    if(!field(sentence, 7, buf, sizeof(buf))) return false;
    data->satellites = (uint8_t)atoi(buf);
    return true;
}
