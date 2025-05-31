#include "gcode_parser.h"
#include <fstream>
#include <sstream>
#include <cctype>

std::vector<GCodeCommand> GCodeParser::parseFile(const std::string& filepath) {
    std::ifstream file(filepath);
    std::string line;
    std::vector<GCodeCommand> commands;

    while (std::getline(file, line)) {
        auto comment_pos = line.find(';');
        if (comment_pos != std::string::npos) {
            line = line.substr(0, comment_pos);
        }
        std::stringstream ss(line);
        std::string token;
        GCodeCommand cmd;

        while (ss >> token) {
            if (token.empty()) {
                continue;
            }
            char letter = std::toupper(token[0]);
            if (letter == ';') {
                break; // rest of line is a comment
            }
            double value = 0.0;
            if (token.size() > 1) {
                try {
                    value = std::stod(token.substr(1));
                } catch (const std::invalid_argument&){
                    continue;
                }
            }
            switch (letter) {
                case 'G': cmd.code = static_cast<int>(value); break;
                case 'X': cmd.latitude = value; cmd.has_lat = true; break;
                case 'Y': cmd.longitude = value; cmd.has_lon = true; break;
                case 'Z': cmd.altitude = value; cmd.has_alt = true; break;
                case 'F': cmd.feed_rate = value; break;
                case 'M':
                    if (static_cast<int>(value) == 100) {
                        cmd.type = GCodeCommand::DropPlow;
                    } else if (static_cast<int>(value) == 101) {
                        cmd.type = GCodeCommand::LiftPlow;
                    }
                    break;
            }
        }

        if (cmd.type == GCodeCommand::Move) {
            if ((cmd.code == 0 || cmd.code == 1) && cmd.has_lat && cmd.has_lon) {
                commands.push_back(cmd);
            }
        } else {
            commands.push_back(cmd);
        }
    }

    return commands;
}
