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
#include <tl/expected.hpp>
#include <type_traits>
namespace OneMotor::Motor::DM {

namespace detail {
inline constexpr uint8_t ENABLE_FRAME_DATA[8] = {0xFF, 0xFF, 0xFF, 0xFF,
                                                 0xFF, 0xFF, 0xFF, 0xFC};
inline constexpr uint8_t DISABLE_FRAME_DATA[8] = {0xFF, 0xFF, 0xFF, 0xFF,
                                                  0xFF, 0xFF, 0xFF, 0xFD};
inline constexpr uint8_t SETZERO_FRAME_DATA[8] = {0xFF, 0xFF, 0xFF, 0xFF,
                                                  0xFF, 0xFF, 0xFF, 0xFE};
inline constexpr uint8_t CLEAN_ERROR_DATA[8] = {0xFF, 0xFF, 0xFF, 0xFF,
                                                0xFF, 0xFF, 0xFF, 0xFB};
} // namespace detail

template <typename Traits, typename Policy = MITPolicy<Traits>>
class DmMotor : public MotorBase<DmMotor<Traits, Policy>, Traits, Policy> {
  public:
    using Base = MotorBase<DmMotor<Traits, Policy>, Traits, Policy>;
    friend Base;
    DmMotor(Can::CanDriver &driver, uint16_t canId, uint16_t masterId,
            Policy policy = {})
        : Base(driver, policy), m_masterId(masterId), m_canId(canId) {

        const auto result = this->m_driver.open().and_then([this] {
            return this->m_driver.registerCallback(
                {m_masterId},
                [&](Can::CanFrame frame) { this->onFeedback(frame); });
        });
        if (!result) {
            panic("Register Callback Failed for DM Motor");
        }
    }

    tl::expected<void, Error> setZeroPosition() {
        return sendControlFrame(detail::SETZERO_FRAME_DATA);
    }

    tl::expected<void, Error> clearError() {
        return sendControlFrame(detail::CLEAN_ERROR_DATA);
    }

  protected:
    tl::expected<void, Error> enableImpl() {
        return sendControlFrame(detail::ENABLE_FRAME_DATA);
    }

    tl::expected<void, Error> disableImpl() {
        return sendControlFrame(detail::DISABLE_FRAME_DATA);
    }

    tl::expected<typename Traits::UserStatusType, Error> getStatusImpl() {
        if (auto result = sendRefreshStatus(); !result)
            return tl::make_unexpected(result.error());
        Thread::sleep_for(std::chrono::milliseconds(1));
        return Traits::UserStatusType::fromPlain(this->m_buffer.readCopy());
    }
    tl::expected<void, Error> afterPosRef() { return update(); }
    tl::expected<void, Error> afterAngRef() { return update(); }
    tl::expected<void, Error> afterTorRef() { return update(); }

    tl::expected<void, Error> afterRefs() { return update(); }

    tl::expected<void, Error> afterPidParams(float kp, float ki, float kd) {
        if constexpr (std::is_same<Policy, MITPolicy<Traits>>()) {
            this->m_policy.m_kp = kp;
            this->m_policy.m_kd = kd;
        }
        return {};
    }

  private:
    tl::expected<void, Error> sendControlFrame(const uint8_t *data) {
        Can::CanFrame frame{};
        if constexpr (std::same_as<Policy, MITPolicy<Traits>>) {
            frame.id = m_canId;
        }
        else if constexpr(std::same_as<Policy, PosVelPolicy<Traits>>) {
            frame.id = m_canId + 0x100;
        }
        else {
            frame.id = m_canId + 0x200;
        }
        frame.dlc = 8;

        memcpy(&frame.data, data, 8);
        return this->m_driver.send(frame);
    };

    tl::expected<void, Error> update() {
        auto status = this->m_buffer.readCopy();
        auto output = this->m_policy.compute(this->m_pos_ref, this->m_ang_ref,
                                             this->m_tor_ref, status);
        return applyOutput(output);
    }

    tl::expected<void, Error> applyOutput(const DmControlOutput &output) {
        switch (output.mode) {
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

    tl::expected<void, Error> sendMITFrame(const DmControlOutput &out) {
        Can::CanFrame frame{};
        frame.id = m_canId;
        frame.dlc = 8;

        std::memcpy(frame.data, out.data, 8);

        return this->m_driver.send(frame);
    }

    tl::expected<void, Error> sendPosVelFrame(const DmControlOutput &out) {
        Can::CanFrame frame{};
        frame.id = m_canId + 0x100;
        frame.dlc = 8;

        std::memcpy(frame.data, out.data, 8);

        return this->m_driver.send(frame);
    }

    tl::expected<void, Error> sendVelFrame(const DmControlOutput &out) {
        Can::CanFrame frame{};
        frame.id = m_canId + 0x200;
        frame.dlc = 4;

        std::memcpy(frame.data, out.data, 4);

        return this->m_driver.send(frame);
    }

    tl::expected<void, Error> sendRefreshStatus() noexcept {
        Can::CanFrame frame{};
        frame.dlc = 4;
        frame.id = 0x7FF;
        auto &data = frame.data;
        const auto *ibuf = reinterpret_cast<const uint8_t *>(&m_canId);
        data[0] = *(ibuf);
        data[1] = *(ibuf + 1);
        data[2] = 0xCC;
        data[3] = 0x00;
        return this->m_driver.send(frame);
    }

    void onFeedback(Can::CanFrame frame) {
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
using J4340_MIT = DmMotor<J4340Traits, MITPolicy<J4340Traits>>;
using J4340_PosVel = DmMotor<J4340Traits, PosVelPolicy<J4340Traits>>;
using J4340_Vel = DmMotor<J4340Traits, VelPolicy<J4340Traits>>;

template <typename Policy = MITPolicy<J8009Traits>>
using J8009 = DmMotor<J8009Traits, Policy>;
using J8009_MIT = DmMotor<J8009Traits, MITPolicy<J8009Traits>>;
using J8009_PosVel = DmMotor<J8009Traits, PosVelPolicy<J8009Traits>>;
using J8009_Vel = DmMotor<J8009Traits, VelPolicy<J8009Traits>>;

template <typename Policy = MITPolicy<J10010LTraits>>
using J10010L = DmMotor<J10010LTraits, Policy>;
using J10010L_MIT = DmMotor<J8009Traits, MITPolicy<J10010LTraits>>;
using J10010L_PosVel = DmMotor<J8009Traits, PosVelPolicy<J10010LTraits>>;
using J10010L_Vel = DmMotor<J8009Traits, VelPolicy<J10010LTraits>>;

} // namespace OneMotor::Motor::DM

#endif // ONE_MOTOR_DM_DMMOTOR_HPP_
