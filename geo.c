#include "geo.h"
#include <math.h>

/*
 * The firmware is compiled with -fsingle-precision-constant, so bare float
 * literals like 2.0 or 180.0 are treated as float and would trigger
 * -Werror=double-promotion when mixed with double variables.
 * Fix: use integer literals (implicitly int→double, not float→double) and
 * explicitly cast M_PI to double.
 */
#define DEG2RAD(x) ((double)(x) * ((double)M_PI / 180))

float geo_distance_m(double lat1, double lon1, double lat2, double lon2) {
    const double R = 6371000; /* Earth mean radius, metres */

    double dlat = DEG2RAD(lat2 - lat1);
    double dlon = DEG2RAD(lon2 - lon1);

    double a = sin(dlat / 2) * sin(dlat / 2) +
               cos(DEG2RAD(lat1)) * cos(DEG2RAD(lat2)) *
               sin(dlon / 2) * sin(dlon / 2);

    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return (float)(R * c);
}
