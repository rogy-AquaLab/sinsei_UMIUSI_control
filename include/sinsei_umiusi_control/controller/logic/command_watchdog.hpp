#ifndef SINSEI_UMIUSI_CONTROL_CONTROLLER_LOGIC_COMMAND_WATCHDOG_HPP
#define SINSEI_UMIUSI_CONTROL_CONTROLLER_LOGIC_COMMAND_WATCHDOG_HPP

#include <atomic>
#include <chrono>
#include <cstdint>

namespace sinsei_umiusi_control::controller::logic {

// 指令が最後に届いてから timeout を過ぎたかを見る。
// 購読のコールバック（executor のスレッド）から feed()、update()（制御ループ）から is_fresh() を呼ぶので、
// 中身は atomic にしてある。
// ROS の時刻ではなく steady_clock を使う（controller_manager から渡る time と時刻源が違うと
// 引き算で例外になるため。経過時間だけが要る）。
class CommandWatchdog {
  public:
    using Clock = std::chrono::steady_clock;

    // timeout <= 0 で無効（常に新しいとみなす = 従来どおり最後の指令を保つ）
    explicit CommandWatchdog(std::chrono::nanoseconds timeout = std::chrono::nanoseconds{0})
    : timeout_ns{static_cast<std::int64_t>(timeout.count())} {}

    auto set_timeout(std::chrono::nanoseconds timeout) -> void {
        this->timeout_ns.store(static_cast<std::int64_t>(timeout.count()));
    }

    auto feed(Clock::time_point now = Clock::now()) -> void {
        this->last_ns.store(
            static_cast<std::int64_t>(now.time_since_epoch().count()), std::memory_order_release);
        this->fed.store(true, std::memory_order_release);
    }

    // 一度も届いていなければ false（無効のときを除く）
    auto is_fresh(Clock::time_point now = Clock::now()) const -> bool {
        const auto timeout = this->timeout_ns.load();
        if (timeout <= 0) {
            return true;
        }
        if (!this->fed.load(std::memory_order_acquire)) {
            return false;
        }
        const auto elapsed = static_cast<std::int64_t>(now.time_since_epoch().count()) -
                             this->last_ns.load(std::memory_order_acquire);
        return elapsed <= timeout;
    }

  private:
    std::atomic<std::int64_t> timeout_ns;
    std::atomic<std::int64_t> last_ns{0};
    std::atomic<bool> fed{false};
};

}  // namespace sinsei_umiusi_control::controller::logic

#endif  // SINSEI_UMIUSI_CONTROL_CONTROLLER_LOGIC_COMMAND_WATCHDOG_HPP
