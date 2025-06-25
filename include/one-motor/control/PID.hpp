#ifndef PID_HPP
#define PID_HPP
#include <type_traits>

#include "one-motor/util/Arithmetic.hpp"
#include "one-motor/util/DeltaT.hpp"

namespace OneMotor::Control
{
    template <Util::Arithmetic ValueType_ = float>
    struct PID_Params
    {
        ValueType_ Kp{1.0f};
        ValueType_ Ki{0.0f};
        ValueType_ Kd{0.0f};
        ValueType_ MaxOutput{std::numeric_limits<ValueType_>::infinity()};
        ValueType_ Deadband{0.0f};
        ValueType_ IntegralLimit{std::numeric_limits<ValueType_>::infinity()};
        ValueType_ DerivativeFilterRC{0.02f};
        ValueType_ OutputFilterRC{0.02f};
    };

    struct Positional
    {
    };

    struct Incremental
    {
    };

    // 特性标签
    struct WithDeadband // 死区
    {
    };

    struct WithIntegralLimit // 积分限幅
    {
    };

    struct WithDerivativeOnMeasurement // 微分先行
    {
    };

    struct WithDerivativeFilter // 微分项滤波
    {
    };

    struct WithOutputFilter // 输出滤波
    {
    };

    struct WithOutputLimit // 输出限幅
    {
    };

    template <typename Algorithm = Positional, Util::Arithmetic ValueType = float, typename... Features>
    class PIDController
    {
        // 特性检查
        static constexpr bool PositionalPID = std::is_same_v<Algorithm, Positional>;
        static constexpr bool HasDeadband = (std::is_same_v<Features, WithDeadband> || ...);
        static constexpr bool HasIntegralLimit = (std::is_same_v<Features, WithIntegralLimit> || ...);
        static constexpr bool HasDerivativeOnMeasurement =
            (std::is_same_v<Features, WithDerivativeOnMeasurement> || ...);
        static constexpr bool HasDerivativeFilter = (std::is_same_v<Features, WithDerivativeFilter> || ...);
        static constexpr bool HasOutputFilter = (std::is_same_v<Features, WithOutputFilter> || ...);
        static constexpr bool HasOutputLimit = (std::is_same_v<Features, WithOutputLimit> || ...);

        // 状态变量
        ValueType ITerm{};
        ValueType prev_error{};
        ValueType prev_prev_error{};
        ValueType prev_ref{};
        ValueType prev_measure{};
        ValueType prev_derivative{};
        ValueType prev_output{};
        Util::DeltaT<ValueType> deltaT{};

    public:
        mutable ValueType MaxOutputVal;
        mutable ValueType DeadbandVal;
        mutable ValueType IntegralLimitVal;
        mutable ValueType Kp;
        mutable ValueType Ki;
        mutable ValueType Kd;
        mutable ValueType D_RC;
        mutable ValueType O_RC;

        explicit PIDController(const PID_Params<ValueType>& params) :
            MaxOutputVal(params.MaxOutput), DeadbandVal(params.Deadband), IntegralLimitVal(params.IntegralLimit),
            Kp(params.Kp), Ki(params.Ki), Kd(params.Kd),
            D_RC(params.DerivativeFilterRC), O_RC(params.OutputFilterRC)
        {
        }

        ValueType compute(ValueType ref, ValueType measure)
        {
            const ValueType error = ref - measure;
            if constexpr (HasDeadband)
            {
                if (std::abs(error) <= DeadbandVal)
                {
                    return prev_output;
                }
            }
            ValueType dt = deltaT.getDeltaMS();
            ValueType P, I{}, D, output;
            if constexpr (PositionalPID)
            {
                P = Kp * error;
                ITerm = Ki * error * dt;
            }
            else
            {
                P = Kp * (error - prev_error);
                ITerm = Ki * error * dt;
            }

            D = [&]
            {
                if constexpr (HasDerivativeOnMeasurement)
                {
                    ValueType meas_deriv = -(measure - prev_measure) / dt;
                    prev_measure = measure;
                    return meas_deriv;
                }
                else if constexpr (PositionalPID)
                {
                    return (error - prev_error) / dt;
                }
                else
                {
                    return (error - 2 * prev_error + prev_prev_error) / dt;
                }
            }();
            D *= Kd;

            // 应用微分滤波
            if constexpr (HasDerivativeFilter)
            {
                D = D * dt / (D_RC + dt) + prev_derivative * D_RC / (D_RC + dt);
            }
            // 应用积分限幅
            if constexpr (HasIntegralLimit)
            {
                ValueType temp_Iout = I + ITerm;
                if (const ValueType temp_Output = P + I + D; abs(temp_Output) > MaxOutputVal)
                {
                    if (error * I > 0)
                    {
                        ITerm = 0;
                    }
                }
                if (temp_Iout > IntegralLimitVal)
                {
                    ITerm = 0;
                    I = IntegralLimitVal;
                }
                if (temp_Iout < -IntegralLimitVal)
                {
                    ITerm = 0;
                    I = -IntegralLimitVal;
                }
            }

            I += ITerm;
            output = P + I + D;

            // 应用输出滤波
            if constexpr (HasOutputFilter)
            {
                output = output * dt / (O_RC + dt) + prev_output * O_RC / (O_RC + dt);
            }

            // 应用输出限幅
            if constexpr (HasOutputLimit)
            {
                output = std::clamp(output, -MaxOutputVal, MaxOutputVal);
            }

            prev_ref = ref;
            prev_derivative = D;
            prev_output = output;
            prev_error = error;

            return output;
        }

        void reset()
        {
            ITerm = ValueType{};
            prev_error = ValueType{};
            prev_prev_error = ValueType{};
            prev_ref = ValueType{};
            prev_measure = ValueType{};
            prev_derivative = ValueType{};
            prev_output = ValueType{};

            deltaT.reset();
        }
    };
}

#endif //PID_HPP
