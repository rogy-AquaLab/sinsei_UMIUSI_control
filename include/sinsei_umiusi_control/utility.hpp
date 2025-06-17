#include <optional>
#include <string>
#include <vector>
#ifndef SINSEI_UMIUSI_CONTROL_UTILITY_HPP
#define SINSEI_UMIUSI_CONTROL_UTILITY_HPP

// ヘルパー関数をまとめておくファイル

namespace sinsei_umiusi_control::utility {

// 特定の名前のLoaned(Command|State)Interfaceを`interfaces`から探し、見つかった場合はその所有権を移す。
template <typename LoanedInterface>
auto find_interface(const std::string & name, std::vector<LoanedInterface> & interfaces)
    -> std::optional<LoanedInterface> {
    auto it = std::find_if(interfaces.begin(), interfaces.end(), [&](const auto & ifc) {
        return ifc.get_name() == name;
    });
    if (it != interfaces.end()) {
        return std::move(*it);
    }
    return std::nullopt;
}

}  // namespace sinsei_umiusi_control::utility

#endif  // SINSEI_UMIUSI_CONTROL_UTILITY_HPP