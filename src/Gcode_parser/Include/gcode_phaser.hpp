#pragma once
#include <string>
#include <vector>

struct GCodeCommand {
    int code; // G0 or G1
    double latitude = 0.0;
    double longitude = 0.0;
    double altitude = 0.0;
    bool has_lat = false, has_lon = false, has_alt = false;
    double feed_rate = 0.0;
};

class GCodeParser {
public:
    std::vector<GCodeCommand> parseFile(const std::string& filepath);
};
