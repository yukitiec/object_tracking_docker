#include "tracker_pkg/utils.h"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>

bool parse_bool(const std::string& s)
{
    std::string v = s;
    for (char& c : v) {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }

    if (v == "true" || v == "1")
        return true;
    if (v == "false" || v == "0")
        return false;

    throw std::runtime_error("Invalid boolean value: " + s);
}

std::string trim(const std::string& s)
{
    const std::string whitespace = " \t\r\n";
    const size_t start = s.find_first_not_of(whitespace);
    if (start == std::string::npos)
        return "";

    const size_t end = s.find_last_not_of(whitespace);
    return s.substr(start, end - start + 1);
}

Config load_config(const std::string& filename)
{
    Config cfg;
    std::ifstream fin(filename);

    if (!fin.is_open()) {
        throw std::runtime_error("Failed to open config file: " + filename);
    }

    std::string line;
    size_t line_number = 0;

    while (std::getline(fin, line))
    {
        ++line_number;

        // Remove comments
        size_t comment_pos = line.find("//");
        if (comment_pos != std::string::npos) {
            line = line.substr(0, comment_pos);
        }

        // Trim whitespace
        line = trim(line);

        // Skip empty lines
        if (line.empty()) {
            continue;
        }

        std::istringstream iss(line);
        std::string key;
        iss >> key;

        if (key == "display")
        {
            std::string value;
            iss >> value;
            if (value.empty()) {
                throw std::runtime_error("Missing value for 'display' at line " + std::to_string(line_number));
            }
            cfg.display = parse_bool(value);
        }
        else if (key == "time_capture")
        {
            if (!(iss >> cfg.time_capture)) {
                throw std::runtime_error("Invalid value for 'time_capture' at line " + std::to_string(line_number));
            }
        }
        else if (key == "video_path")
        {
            std::string value;
            std::getline(iss, value);
            value = trim(value);

            if (value.empty()) {
                throw std::runtime_error("Missing value for 'video_path' at line " + std::to_string(line_number));
            }

            cfg.video_path = value;
        }
        else if (key == "yolo_path")
        {
            std::string value;
            std::getline(iss, value);
            value = trim(value);

            if (value.empty()) {
                throw std::runtime_error("Missing value for 'yolo_path' at line " + std::to_string(line_number));
            }

            cfg.yolo_path = value;
        }
        else if (key == "yoloWidth")
        {
            if (!(iss >> cfg.yoloWidth)) {
                throw std::runtime_error("Invalid value for 'yoloWidth' at line " + std::to_string(line_number));
            }
        }
        else if (key == "yoloHeight")
        {
            if (!(iss >> cfg.yoloHeight)) {
                throw std::runtime_error("Invalid value for 'yoloHeight' at line " + std::to_string(line_number));
            }
        }
        else if (key == "object_index")
        {
            std::string values;
            std::getline(iss, values);
            values = trim(values);

            if (values.empty()) {
                throw std::runtime_error("Missing value for 'object_index' at line " + std::to_string(line_number));
            }

            cfg.object_index.clear();

            std::stringstream ss(values);
            std::string token;
            while (std::getline(ss, token, ','))
            {
                token = trim(token);
                if (!token.empty()) {
                    cfg.object_index.push_back(static_cast<size_t>(std::stoull(token)));
                }
            }
        }
        else if (key == "IoU_threshold")
        {
            if (!(iss >> cfg.IoU_threshold)) {
                throw std::runtime_error("Invalid value for 'IoU_threshold' at line " + std::to_string(line_number));
            }
        }
        else if (key == "conf_threshold")
        {
            if (!(iss >> cfg.conf_threshold)) {
                throw std::runtime_error("Invalid value for 'conf_threshold' at line " + std::to_string(line_number));
            }
        }
        else
        {
            std::cerr << "Warning: Unknown config key at line "
                      << line_number << ": " << key << std::endl;
        }
    }

    return cfg;
}