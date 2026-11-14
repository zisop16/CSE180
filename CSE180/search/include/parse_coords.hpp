#include <fstream>
#include <iostream>
#include <regex>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>
 
std::vector<std::tuple<double, double>> load_coordinates(const std::string& filename) {
    std::ifstream in(filename);
    if (!in) {
        throw std::runtime_error("Could not open file: " + filename);
    }
 
    // Matches "(x, y)" with optional whitespace, signs, decimals, exponents.
    // Handles bare decimals like ".179" and "-.24".
    static const std::string num = R"([+-]?(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?)";
    static const std::regex pair_re(R"(\(\s*()" + num + R"()\s*,\s*()" + num + R"()\s*\))");
 
    std::vector<std::tuple<double, double>> coords;
    std::string line;
 
    while (std::getline(in, line)) {
        // Skip leading whitespace so indented lines still count.
        const auto first = line.find_first_not_of(" \t\r");
        if (first == std::string::npos || line[first] != '(') continue;
 
        // A line may hold several pairs, e.g. "(1,2), (3,4)".
        for (std::sregex_iterator it(line.begin() + first, line.end(), pair_re), end;
             it != end; ++it) {
            coords.emplace_back(std::stod((*it)[1]), std::stod((*it)[2]));
        }
    }
 
    return coords;
}