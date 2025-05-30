#include "coordinate_conversion.h"
#include <cmath>

namespace coordinate_conversion {
static constexpr double R_EARTH = 6378137.0; // WGS84 radius in meters

ENUCoord latLonToLocal(double lat_deg, double lon_deg,
                       double origin_lat_deg, double origin_lon_deg)
{
    double lat_rad = lat_deg * M_PI / 180.0;
    double lon_rad = lon_deg * M_PI / 180.0;
    double origin_lat_rad = origin_lat_deg * M_PI / 180.0;
    double origin_lon_rad = origin_lon_deg * M_PI / 180.0;

    double dlat = lat_rad - origin_lat_rad;
    double dlon = lon_rad - origin_lon_rad;

    double east  = dlon * R_EARTH * std::cos(origin_lat_rad);
    double north = dlat * R_EARTH;
    return {east, north};
}

} // namespace coordinate_conversion
