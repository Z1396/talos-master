#pragma once

#include <string>

struct Options {
    std::string device     = "/dev/ttyS4";
    int         baud       = 115200;
    double      duration_s = 0.0;
    double      send_hz    = 0.0;
    bool        list_only  = false;
};

Options parse_args(int argc, char* argv[]);
void print_usage(const char* prog);