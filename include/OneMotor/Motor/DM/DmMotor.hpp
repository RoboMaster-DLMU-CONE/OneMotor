#ifndef ONE_MOTOR_DM_DMMOTOR_HPP_
#define ONE_MOTOR_DM_DMMOTOR_HPP_

#include "DmPolicy.hpp"
#include "DmTraits.hpp"
#include <OneMotor/Can/CanFrame.hpp>
#include <OneMotor/Motor/DM/DmFrame.hpp>
#include <OneMotor/Motor/MotorBase.hpp>
#include <OneMotor/Thread/Othread.hpp>
#include <OneMotor/Util/Error.hpp>
#include <OneMotor/Util/Panic.hpp>
#include <cstdint>
#include <cstring>
#include <string>
#include <tl/expected.hpp>
#include <type_traits>
#include <utility>

namespace OneMotor::Motor::DM
{
    namespace detail
    {
        inline constexpr uint8_t ENABLE_FRAME_DATA[8] = {
            0xFF, 0xFF, 0xFF, 0xFF,
            0xFF, 0xFF, 0xFF, 0xFC
        };
        inline constexpr uint8_t DISABLE_FRAME_DATA[8] = {
            0xFF, 0xFF, 0xFF, 0xFF,
            0xFF, 0xFF, 0xFF, 0xFD
        };
        inline constexpr uint8_t SETZERO_FRAME_DATA[8] = {
            0xFF, 0xFF, 0xFF, 0xFF,
            0xFF, 0xFF, 0xFF, 0xFE
        };
        inline constexpr uint8_t CLEAN_ERROR_DATA[8] = {
            0xFF, 0xFF, 0xFF, 0xFF,
            0xFF, 0xFF, 0xFF, 0xFB
        };
    } // namespace detail

    template <typename Traits, typename Policy = MITPolicy<Traits>>
    class DmMotor : public MotorBase<DmMotor<Traits, Policy>, Traits, Policy>
    {
    public:
        using Base = MotorBase<DmMotor<Traits, Policy>, Traits, Policy>;
        friend Base;
        DmMotor() = default;

        DmMotor(Can::CanDriver& driver, uint16_t canId, uint16_t masterId,
                Policy policy = {})
        {
            auto result =
                init(driver, canId, masterId, std::move(policy));
            if (!result)
            {
                panic(result.error().message);
            }
        }

        tl::expected<void, Error> init(Can::CanDriver& driver, const uint16_t canId,
                                       const uint16_t masterId, Policy policy)
        {
            if (auto base_result = Base::init(driver, std::move(policy));
                !base_result)
            {
                return base_result;
            }

            m_canId = canId;
            m_masterId = masterId;
            auto driver_result = this->driver();
            if (!driver_result)
            {
                return tl::make_unexpected(driver_result.error());
            }
            auto& driver_ref = driver_result->get();
            auto result = driver_ref.open().and_then([this, &driver_ref]
            {
                return driver_ref.registerCallback(
                    {m_masterId},
                    [this](Can::CanFrame frame) { this->onFeedback(frame); });
            });

            if (!result)
            {
                this->resetInitialization();
            }
            return result;
        }

        tl::expected<void, Error> setZeroPosition()
        {
            return sendControlFrame(detail::SETZERO_FRAME_DATA);
        }

        tl::expected<void, Error> clearError()
        {
            return sendControlFrame(detail::CLEAN_ERROR_DATA);
        }

        tl::expected<void, Error> sendRefreshStatus()
        {
            Can::CanFrame frame{};
            frame.dlc = 4;
            frame.id = 0x7FF;
            auto& data = frame.data;
            const auto* ibuf = reinterpret_cast<const uint8_t*>(&m_canId);
            data[0] = *(ibuf);
            data[1] = *(ibuf + 1);
            data[2] = 0xCC;
            data[3] = 0x00;
            auto driver_result = this->driver();
            if (!driver_result)
            {
                return tl::make_unexpected(driver_result.error());
            }
            auto& driver_ref = driver_result->get();
            return driver_ref.send(frame);
        }

    protected:
        tl::expected<void, Error> enableImpl() final
        {
            return sendControlFrame(detail::ENABLE_FRAME_DATA);
        }

        tl::expected<void, Error> disableImpl() final
        {
            return sendControlFrame(detail::DISABLE_FRAME_DATA);
        }

        tl::expected<typename Traits::UserStatusType, Error> getStatusImpl()
        final
        {
            return Traits::UserStatusType::fromPlain(this->m_buffer.readCopy());
        }

        tl::expected<void, Error> afterPosRef() final { return update(); }
        tl::expected<void, Error> afterAngRef() final { return update(); }
        tl::expected<void, Error> afterTorRef() final { return update(); }

        tl::expected<void, Error> afterRefs() final { return update(); }

        tl::expected<void, Error> afterPidParams(float kp, float ki,
                                                 float kd) final
        {
            if constexpr (std::is_same<Policy, MITPolicy<Traits>>())
            {
                auto policy_result = this->getPolicy();
                if (!policy_result)
                {
                    return tl::make_unexpected(policy_result.error());
                }
                auto& policy = policy_result->get();
                policy.m_kp = kp;
                policy.m_kd = kd;
            }
            return {};
        }

    private:
        tl::expected<void, Error> sendControlFrame(const uint8_t* data)
        {
            Can::CanFrame frame{};
            if constexpr (std::same_as<Policy, MITPolicy<Traits>>)
            {
                frame.id = m_canId;
            }
            else if constexpr (std::same_as<Policy, PosVelPolicy<Traits>>)
            {
                frame.id = m_canId + 0x100;
            }
            else
            {
                frame.id = m_canId + 0x200;
            }
            frame.dlc = 8;

            memcpy(&frame.data, data, 8);
            auto driver_result = this->driver();
            if (!driver_result)
            {
                return tl::make_unexpected(driver_result.error());
            }
            auto& driver_ref = driver_result->get();
            return driver_ref.send(frame);
        };

        tl::expected<void, Error> update()
        {
            auto status = this->m_buffer.readCopy();
            auto policy_result = this->getPolicy();
            if (!policy_result)
            {
                return tl::make_unexpected(policy_result.error());
            }
            auto& policy = policy_result->get();
            auto output = policy.compute(this->m_pos_ref, this->m_ang_ref,
                                         this->m_tor_ref, status);
            return applyOutput(output);
        }

        tl::expected<void, Error> applyOutput(const DmControlOutput& output)
        {
            switch (output.mode)
            {
            case DmControlOutput::Mode::MIT:
                return sendMITFrame(output);
            case DmControlOutput::Mode::PosVel:
                return sendPosVelFrame(output);
            case DmControlOutput::Mode::Vel:
                return sendVelFrame(output);
            default:
                return {};
            }
        }

        tl::expected<void, Error> sendMITFrame(const DmControlOutput& out)
        {
            Can::CanFrame frame{};
            frame.id = m_canId;
            frame.dlc = 8;

            std::memcpy(frame.data, out.data, 8);

            auto driver_result = this->driver();
            if (!driver_result)
            {
                return tl::make_unexpected(driver_result.error());
            }
            auto& driver_ref = driver_result->get();
            return driver_ref.send(frame);
        }

        tl::expected<void, Error> sendPosVelFrame(const DmControlOutput& out)
        {
            Can::CanFrame frame{};
            frame.id = m_canId + 0x100;
            frame.dlc = 8;

            std::memcpy(frame.data, out.data, 8);

            auto driver_result = this->driver();
            if (!driver_result)
            {
                return tl::make_unexpected(driver_result.error());
            }
            auto& driver_ref = driver_result->get();
            return driver_ref.send(frame);
        }

        tl::expected<void, Error> sendVelFrame(const DmControlOutput& out)
        {
            Can::CanFrame frame{};
            frame.id = m_canId + 0x200;
            frame.dlc = 4;

            std::memcpy(frame.data, out.data, 4);

            auto driver_result = this->driver();
            if (!driver_result)
            {
                return tl::make_unexpected(driver_result.error());
            }
            auto& driver_ref = driver_result->get();
            return driver_ref.send(frame);
        }

        void onFeedback(Can::CanFrame frame)
        {
            this->m_buffer.push(
                typename Traits::StatusType(frame, TypeTag<Traits>{}));
        }

        uint16_t m_masterId{}, m_canId{};
    };

    template <typename Policy = MITPolicy<J4310Traits>>
    using J4310 = DmMotor<J4310Traits, Policy>;

    using J4310_MIT = DmMotor<J4310Traits>;
    using J4310_PosVel = DmMotor<J4310Traits, PosVelPolicy<J4310Traits>>;
    using J4310_Vel = DmMotor<J4310Traits, VelPolicy<J4310Traits>>;

    template <typename Policy = MITPolicy<J4340Traits>>
    using J4340 = DmMotor<J4340Traits, Policy>;
    using J4340_MIT = DmMotor<J4340Traits>;
    using J4340_PosVel = DmMotor<J4340Traits, PosVelPolicy<J4340Traits>>;
    using J4340_Vel = DmMotor<J4340Traits, VelPolicy<J4340Traits>>;

    template <typename Policy = MITPolicy<J8009Traits>>
    using J8009 = DmMotor<J8009Traits, Policy>;
    using J8009_MIT = DmMotor<J8009Traits>;
    using J8009_PosVel = DmMotor<J8009Traits, PosVelPolicy<J8009Traits>>;
    using J8009_Vel = DmMotor<J8009Traits, VelPolicy<J8009Traits>>;

    template <typename Policy = MITPolicy<J10010LTraits>>
    using J10010L = DmMotor<J10010LTraits, Policy>;
    using J10010L_MIT = DmMotor<J10010LTraits>;
    using J10010L_PosVel = DmMotor<J10010LTraits, PosVelPolicy<J10010LTraits>>;
    using J10010L_Vel = DmMotor<J10010LTraits, VelPolicy<J10010LTraits>>;
} // namespace OneMotor::Motor::DM

#endif // ONE_MOTOR_DM_DMMOTOR_HPP_
