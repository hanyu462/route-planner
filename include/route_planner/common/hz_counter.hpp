#pragma once

#include <chrono>
#include <cstdint>

namespace route_planner::common {

class HzCounter {
public:
    void tick() {
        ++count_;
        const auto  now     = std::chrono::steady_clock::now();
        const float elapsed = std::chrono::duration<float>(now - last_).count();
        if (elapsed >= 1.0f) {
            hz_    = static_cast<float>(count_) / elapsed;
            count_ = 0;
            last_  = now;
        }
    }

    float hz() const { return hz_; }

private:
    uint32_t count_ = 0;
    float    hz_    = 0.0f;
    std::chrono::steady_clock::time_point last_ = std::chrono::steady_clock::now();
};

}  // namespace route_planner::common
