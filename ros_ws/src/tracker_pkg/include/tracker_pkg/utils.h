#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <stdexcept>
#include <cctype>
#include "tracker_pkg/global_parameters.h"

bool parse_bool(const std::string& s);

std::string trim(const std::string& s);

Config load_config(const std::string& filename);