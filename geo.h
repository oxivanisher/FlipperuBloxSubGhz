#pragma once

/**
 * Calculate great-circle distance in metres between two WGS-84 coordinates
 * using the Haversine formula.
 *
 * @param lat1 Latitude  of point 1 (decimal degrees, positive = North)
 * @param lon1 Longitude of point 1 (decimal degrees, positive = East)
 * @param lat2 Latitude  of point 2
 * @param lon2 Longitude of point 2
 * @return Distance in metres
 */
float geo_distance_m(double lat1, double lon1, double lat2, double lon2);
