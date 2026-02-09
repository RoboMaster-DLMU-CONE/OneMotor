#ifndef ONEMOTOR_DJIPARAM_HPP
#define ONEMOTOR_DJIPARAM_HPP
#include <cstdint>
#include <one/PID/PidChain.hpp>
#include <one/PID/PidController.hpp>
#include <one/PID/PidParams.hpp>
#include <variant>

namespace one::motor::dji
{
    using PIDFeatures =
    pid::FeaturePack<pid::WithDeadband, pid::WithIntegralLimit,
                     pid::WithOutputLimit, pid::WithOutputFilter,
                     pid::WithDerivativeFilter>;

    struct AngMode
    {
        explicit AngMode(const pid::PidParams<float>& pid_param)
            : pid_controller(pid_param)
        {
        }

        pid::PidController<pid::Positional, float, PIDFeatures> pid_controller;
    };

    struct PosAngMode
    {
        explicit PosAngMode(const pid::PidParams<float>& pos_param,
                            const pid::PidParams<float>& ang_param)
            : pos_controller(pos_param), ang_controller(ang_param)
        {
            pid_chain.add(pos_controller);
            pid_chain.add(ang_controller);
        }

        pid::PidController<pid::Positional, float, PIDFeatures> pos_controller;
        pid::PidController<pid::Positional, float, PIDFeatures> ang_controller;
        pid::PidChain<float> pid_chain;
    };

    struct MITMode
    {
        float Kp{};
        float Kd{};
    };

    using Mode = std::variant<AngMode, PosAngMode, MITMode>;

    struct Param
    {
        uint8_t id;
        Mode mode;
    };
} // namespace one::motor::dji

#endif // ONEMOTOR_DJIPARAM_HPP
