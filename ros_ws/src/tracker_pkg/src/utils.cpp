#include "tracker_pkg/utils.h"

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
    while (std::getline(fin, line))
    {
        // Remove comments
        size_t comment_pos = line.find("//");
        if (comment_pos != std::string::npos) {
            line = line.substr(0, comment_pos);
        }

        // Trim whitespace
        line = trim(line);

        // Skip empty lines
        if (line.empty())
            continue;

        std::istringstream iss(line);
        std::string key;
        iss >> key;

        if (key == "display")
        {
            std::string value;
            iss >> value;
            cfg.display = parse_bool(value);
        }
        else if (key == "time_capture")
        {
            iss >> cfg.time_capture;
        }
        else if (key == "yolo_path")
        {
            std::string value;
            std::getline(iss, value);
            cfg.yolo_path = trim(value);
        }
        else if (key == "yoloWidth")
        {
            iss >> cfg.yoloWidth;
        }
        else if (key == "yoloHeight")
        {
            iss >> cfg.yoloHeight;
        }
        else if (key == "object_index")
        {
            std::string values;
            iss >> values;

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
            iss >> cfg.IoU_threshold;
        }
        else if (key == "conf_threshold")
        {
            iss >> cfg.conf_threshold;
        }
        else
        {
            std::cerr << "Warning: Unknown config key: " << key << std::endl;
        }
    }

    return cfg;
}