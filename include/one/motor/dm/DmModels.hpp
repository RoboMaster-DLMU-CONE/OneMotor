#ifndef ONE_MOTOR_DM_DMTRAITS_HPP_
#define ONE_MOTOR_DM_DMTRAITS_HPP_

namespace one::motor::dm {

struct J4310Model {
    static constexpr float V_MIN = -45.0f; // rad/s
    static constexpr float V_MAX = 45.0f;
    static constexpr float P_MIN = -12.5f; // rad
    static constexpr float P_MAX = 12.5f;
    static constexpr float T_MIN = -18.0f; // N*m
    static constexpr float T_MAX = 18.0f;
    static constexpr float KP_MIN = 0.0f;
    static constexpr float KP_MAX = 500.0f;
    static constexpr float KD_MIN = 0.0f;
    static constexpr float KD_MAX = 5.0f;
};

struct J4340Model {
    static constexpr float V_MIN = -10.0f; // rad/s
    static constexpr float V_MAX = 10.0f;
    static constexpr float P_MIN = -12.5f; // rad
    static constexpr float P_MAX = 12.5f;
    static constexpr float T_MIN = -28.0f; // N*m
    static constexpr float T_MAX = 28.0f;
    static constexpr float KP_MIN = 0.0f;
    static constexpr float KP_MAX = 500.0f;
    static constexpr float KD_MIN = 0.0f;
    static constexpr float KD_MAX = 5.0f;
};

struct J8009Model {
    static constexpr float V_MIN = -45.0f; // rad/s
    static constexpr float V_MAX = 45.0f;
    static constexpr float P_MIN = -12.5f; // rad
    static constexpr float P_MAX = 12.5f;
    static constexpr float T_MIN = -54.0f; // N*m
    static constexpr float T_MAX = 54.0f;
    static constexpr float KP_MIN = 0.0f;
    static constexpr float KP_MAX = 500.0f;
    static constexpr float KD_MIN = 0.0f;
    static constexpr float KD_MAX = 5.0f;
};
struct J10010LModel {
    static constexpr float V_MIN = -25.0f; // rad/s
    static constexpr float V_MAX = 25.0f;
    static constexpr float P_MIN = -12.5f; // rad
    static constexpr float P_MAX = 12.5f;
    static constexpr float T_MIN = -200.0f; // N*m
    static constexpr float T_MAX = 200.0f;
    static constexpr float KP_MIN = 0.0f;
    static constexpr float KP_MAX = 500.0f;
    static constexpr float KD_MIN = 0.0f;
    static constexpr float KD_MAX = 5.0f;
};
} // namespace one::motor::dm

#endif // ONE_MOTOR_DM_DMTRAITS_HPP_
