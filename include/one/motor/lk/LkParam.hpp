#ifndef ONEMOTOR_lkPARAM_HPP
#define ONEMOTOR_lkPARAM_HPP

#include <algorithm>
#include <cstdint>
#include <variant>

namespace one::motor::lk {

struct AngMode {};
struct PosAngMode {};
struct MITMode {
    float Kp{};
    float Kd{};
};

using Mode = std::variant<AngMode, PosAngMode, MITMode>;

struct Param {

    uint16_t can_id{};
    uint16_t master_id{};
    Mode mode{MITMode{}};
};

} // namespace one::motor::lk

#endif // ONEMOTOR_lkPARAM_HPP
