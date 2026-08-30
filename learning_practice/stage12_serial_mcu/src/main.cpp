#include "options.hpp"
#include "utils.hpp"
#include "serial_test.hpp"

int main(int argc, char* argv[]) {
    Options opt = parse_args(argc, argv);

    if (opt.list_only) {
        list_serial_devices();
        return 0;
    }

    return run_serial_test(opt);
}