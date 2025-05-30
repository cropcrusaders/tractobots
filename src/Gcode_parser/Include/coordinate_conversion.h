#pragma once
#include <utility>

namespace coordinate_conversion {
struct ENUCoord {
    double east;
    double north;
};

ENUCoord latLonToLocal(double lat_deg, double lon_deg,
                       double origin_lat_deg, double origin_lon_deg);
}
