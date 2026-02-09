#ifndef ONEMOTOR_DMPARAM_HPP
#define ONEMOTOR_DMPARAM_HPP

#include <algorithm>
#include <cstdint>
#include <variant>

namespace one::motor::dm {

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

} // namespace one::motor::dm

#endif // ONEMOTOR_DMPARAM_HPP
