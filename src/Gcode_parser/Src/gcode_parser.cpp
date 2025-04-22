#include "gcode_parser.h"
#include <fstream>
#include <sstream>
#include <cctype>

std::vector<GCodeCommand> GCodeParser::parseFile(const std::string& filepath) {
    std::ifstream file(filepath);
    std::string line;
    std::vector<GCodeCommand> commands;

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string token;
        GCodeCommand cmd;

        while (ss >> token) {
            char letter = std::toupper(token[0]);
            double value = std::stod(token.substr(1));
            switch (letter) {
                case 'G': cmd.code = static_cast<int>(value); break;
                case 'X': cmd.latitude = value; cmd.has_lat = true; break;
                case 'Y': cmd.longitude = value; cmd.has_lon = true; break;
                case 'Z': cmd.altitude = value; cmd.has_alt = true; break;
                case 'F': cmd.feed_rate = value; break;
            }
        }

        if ((cmd.code == 0 || cmd.code == 1) && cmd.has_lat && cmd.has_lon) {
            commands.push_back(cmd);
        }
    }

    return commands;
}
