#include "io/io.hpp"

#include <iostream>
#include <ostream>

namespace practice::io {

void print_line(std::string_view prefix, std::string_view msg) {
    std::cout << "[" << prefix << "] " << msg << "\n";
}

std::string read_input(std::string_view prompt) {
    std::cout << prompt;
    std::string line;
    std::getline(std::cin, line);
    return line;
}

}  // namespace practice::io
