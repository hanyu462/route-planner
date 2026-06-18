#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <utility>

namespace route_planner::common {

template <typename T>
class LatestBuffer {
public:
    struct Snapshot {
        std::shared_ptr<const T> value;
        std::uint64_t sequence = 0;
    };

    void write(T value)
    {
        auto next = std::make_shared<T>(std::move(value));

        std::lock_guard<std::mutex> lock(mutex_);
        latest_ = std::move(next);
        ++sequence_;
    }

    std::optional<Snapshot> read() const
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!latest_) {
            return std::nullopt;
        }

        return Snapshot{latest_, sequence_};
    }

    std::optional<Snapshot> read_if_new(
        std::uint64_t last_sequence) const
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!latest_ || sequence_ == last_sequence) {
            return std::nullopt;
        }

        return Snapshot{latest_, sequence_};
    }

    bool empty() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return !latest_;
    }

private:
    mutable std::mutex mutex_;

    std::shared_ptr<const T> latest_;
    std::uint64_t sequence_ = 0;
};

}  // namespace route_planner::common
