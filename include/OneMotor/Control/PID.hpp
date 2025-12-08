/**
 * @file PID.hpp
 * @brief 定义了一个可配置的PID控制器。
 * @author MoonFeather
 * @date 2025-06-26
 */
#ifndef PID_HPP
#define PID_HPP
#include <type_traits>
#include <limits>
#include <cmath>
#include <algorithm>

#include "OneMotor/Util/Arithmetic.hpp"
#include "OneMotor/Util/DeltaT.hpp"

namespace OneMotor::Control
{
    /**
     * @brief PID控制器的参数。
     * @tparam ValueType_ PID计算所使用的数据类型。
     */
    template <Arithmetic ValueType_ = float>
    struct PID_Params
    {
        ValueType_ Kp{}; ///< 比例增益
        ValueType_ Ki{}; ///< 积分增益
        ValueType_ Kd{}; ///< 微分增益
#ifdef ONE_MOTOR_LINUX
        ValueType_ MaxOutput{std::numeric_limits<ValueType_>::infinity()}; ///< 最大输出值
#else
        //TODO: 等Zephyr SDK更新看能不能修bug
        ValueType_ MaxOutput{};
#endif

        ValueType_ Deadband{}; ///< 死区范围
#ifdef ONE_MOTOR_LINUX
        ValueType_ IntegralLimit{std::numeric_limits<ValueType_>::infinity()}; ///< 积分项限幅
#else
        ValueType_ IntegralLimit{};
#endif
        ValueType_ DerivativeFilterRC{}; ///< 微分项滤波器的时间常数 (RC)
        ValueType_ OutputFilterRC{}; ///< 输出滤波器的时间常数 (RC)
    };

    /**
     * @brief 位置式PID算法标签。
     */
    struct Positional
    {
    };

    /**
     * @brief 增量式PID算法标签。
     */
    struct Incremental
    {
    };

    // 特性标签
    /** @brief 死区特性标签。如果误差的绝对值小于死区值，则PID输出保持不变。 */
    struct WithDeadband // 死区
    {
    };

    /** @brief 积分限幅特性标签。限制积分项的累积范围。 */
    struct WithIntegralLimit // 积分限幅
    {
    };

    /** @brief 微分先行特性标签 (Derivative on Measurement)。微分项基于测量值的变化率，而不是误差的变化率。 */
    struct WithDerivativeOnMeasurement // 微分先行
    {
    };

    /** @brief 微分项滤波特性标签。为微分项添加一个低通滤波器。 */
    struct WithDerivativeFilter // 微分项滤波
    {
    };

    /** @brief 输出滤波特性标签。为最终输出添加一个低通滤波器。 */
    struct WithOutputFilter // 输出滤波
    {
    };

    /** @brief 输出限幅特性标签。限制PID控制器的最终输出范围。 */
    struct WithOutputLimit // 输出限幅
    {
    };

    /**
     * @brief 特性包容器，用于将多个特性打包成一个类型。
     * @tparam Fs 特性列表
     */
    template <typename... Fs>
    struct FeaturePack
    {
    };

    namespace Detail
    {
        /**
         * @brief 递归检查特性是否存在。
         * @tparam Target 要查找的目标特性 (如 WithDeadband)
         * @tparam T 当前检查的类型 (可能是 FeaturePack，也可能是普通类型)
         */
        template <typename Target, typename T>
        struct HasFeatureCheck : std::is_same<Target, T>
        {
        };

        // 特化：如果当前类型是 FeaturePack，则递归检查包内的所有类型
        template <typename Target, typename... Fs>
        struct HasFeatureCheck<Target, FeaturePack<Fs...>>
        {
            // 使用折叠表达式递归展开
            static constexpr bool value = (HasFeatureCheck<Target, Fs>::value || ...);
        };
    }

    /**
     * @brief 一个通用的、可配置的PID控制器。
     * @tparam Algorithm PID算法类型，可以是 `Positional` (位置式) 或 `Incremental` (增量式)。
     * @tparam ValueType PID计算所使用的数据类型。
     * @tparam Features 一个可变参数模板，用于指定PID控制器的附加特性，例如 `WithDeadband`, `WithIntegralLimit` 等。
     *
     * @details
     * 该PID控制器通过模板参数高度可定制。
     * - `Algorithm` 决定了是使用位置式还是增量式PID。
     * - `ValueType` 决定了内部计算的精度。
     * - `Features` 允许你通过添加特性标签来启用或禁用特定功能，例如：
     *   - `WithDeadband`: 引入死区，当误差很小时忽略不计。
     *   - `WithIntegralLimit`: 防止积分饱和。
     *   - `WithDerivativeOnMeasurement`: 避免因目标值突变引起的微分冲击。
     *   - `WithDerivativeFilter`: 平滑微分项。
     *   - `WithOutputFilter`: 平滑最终输出。
     *   - `WithOutputLimit`: 限制输出范围。
     */
    template <typename Algorithm = Positional, Arithmetic ValueType = float, typename... Features>
    class PIDController
    {
        template <typename TargetFeature>
        static constexpr bool Check = (Detail::HasFeatureCheck<TargetFeature, Features>::value || ...);

        // 特性检查
        static constexpr bool PositionalPID = std::is_same_v<Algorithm, Positional>;

        static constexpr bool HasDeadband = Check<WithDeadband>;
        static constexpr bool HasIntegralLimit = Check<WithIntegralLimit>;
        static constexpr bool HasDerivativeOnMeasurement = Check<WithDerivativeOnMeasurement>;
        static constexpr bool HasDerivativeFilter = Check<WithDerivativeFilter>;
        static constexpr bool HasOutputFilter = Check<WithOutputFilter>;
        static constexpr bool HasOutputLimit = Check<WithOutputLimit>;

        // 状态变量
        ValueType ITerm{}; ///< 积分项累加值
        ValueType prev_error{}; ///< 上一次的误差
        ValueType prev_prev_error{}; ///< 上上次的误差 (用于增量式PID)
        ValueType prev_ref{}; ///< 上一次的目标值
        ValueType prev_measure{}; ///< 上一次的测量值 (用于微分先行)
        ValueType prev_derivative{}; ///< 上一次的微分项输出 (用于滤波)
        ValueType prev_output{}; ///< 上一次的PID总输出 (用于滤波)
        DeltaT<ValueType> deltaT{}; ///< 用于计算时间间隔 `dt`

    public:
        using ParamsType = PID_Params<ValueType>;
        using ValType = ValueType;
        mutable ValueType MaxOutputVal; ///< 最大输出值
        mutable ValueType DeadbandVal; ///< 死区值
        mutable ValueType IntegralLimitVal; ///< 积分限幅值
        mutable ValueType Kp; ///< 比例增益
        mutable ValueType Ki; ///< 积分增益
        mutable ValueType Kd; ///< 微分增益
        mutable ValueType D_RC; ///< 微分滤波器时间常数
        mutable ValueType O_RC; ///< 输出滤波器时间常数

        /**
         * @brief PID控制器的构造函数。
         * @param params 包含所有PID参数的结构体。
         */
        explicit PIDController(const PID_Params<ValueType>& params) :
            MaxOutputVal(params.MaxOutput), DeadbandVal(params.Deadband), IntegralLimitVal(params.IntegralLimit),
            Kp(params.Kp), Ki(params.Ki), Kd(params.Kd),
            D_RC(params.DerivativeFilterRC), O_RC(params.OutputFilterRC)
        {
        }

        /**
         * @brief 计算PID输出。
         * @param ref 目标值 (设定值)。
         * @param measure 实际测量值 (反馈值)。
         * @return PID控制器的输出值。
         */
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

        /**
         * @brief 重置PID控制器的内部状态。
         * @details 将积分项、历史误差、历史输出等所有内部状态变量清零。
         */
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
