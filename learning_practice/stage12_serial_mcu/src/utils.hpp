#pragma once

#include "options.hpp"    // 加上
#include "stats.hpp"      // 加上

void list_serial_devices();
void print_report(const Options& opt, const RxStats& stats,
                  double duration_s, double avg_fps, double avg_dt);