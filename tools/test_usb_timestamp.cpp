// Fixed OS clock samples exercise the real driver's monotonic -> epoch helper.
#include "usb_cam/utils.hpp"
#include <cstdint>
#include <iostream>

static timeval realtime_sample{};
static timespec monotonic_sample{};
extern "C" int __wrap_gettimeofday(timeval *value, void *) noexcept {
    *value = realtime_sample;
    return 0;
}
extern "C" int __wrap_clock_gettime(clockid_t clock, timespec *value) noexcept {
    if (clock != CLOCK_MONOTONIC) return -1;
    *value = monotonic_sample;
    return 0;
}

int main() {
    int cases = 0;
    for (const int64_t fraction : {1, 999, 123456, 567891, 999999}) {
        realtime_sample = {1700000000, fraction};
        monotonic_sample = {1234, 567891000};
        const timeval buffer{1234, 567891};
        const auto image = usb_cam::utils::calc_img_timestamp(
            buffer, usb_cam::utils::get_epoch_time_shift_us());
        if (image.tv_sec != realtime_sample.tv_sec || image.tv_nsec != fraction*1000) {
            std::cerr << "source-clock mapping lost fractional epoch time: " << fraction << '\n';
            return 1;
        }
        ++cases;
    }
    std::cout << "PASS " << cases << " fixed source-clock mappings\n";
    return 0;
}
