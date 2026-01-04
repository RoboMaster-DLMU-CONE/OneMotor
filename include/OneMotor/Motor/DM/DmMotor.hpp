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

template <typename Traits = DmTraits, typename Policy = MITPolicy<Traits>>
class DmMotor : public MotorBase<DmMotor<Traits, Policy>, Traits, Policy> {
  public:
    using Base = MotorBase<DmMotor<Traits, Policy>, Traits, Policy>;
    friend Base;
    DmMotor(Can::CanDriver &driver, uint16_t canId, uint16_t masterId,
            Policy policy = {})
        : Base(driver, policy), m_masterId(masterId), m_canId(canId) {

        const auto result = this->m_driver.open().and_then([this] {
            return this->m_driver.registerCallback(
                {m_masterId}, [&](Can::CanFrame &&frame) {
                    this->onFeedback(std::move(frame));
                });
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

  private:
    tl::expected<void, Error> sendControlFrame(const uint8_t *data) {
        Can::CanFrame frame{};
        frame.id = m_canId + 0x100;
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

        const uint16_t pos =
            float_to_uint(out.position, Traits::P_MAX, Traits::P_MIN, 16);
        const uint16_t vel =
            float_to_uint(out.angular, Traits::V_MAX, Traits::V_MIN, 12);
        const uint16_t tor =
            float_to_uint(out.torque, Traits::T_MAX, Traits::T_MIN, 12);
        const uint16_t kp =
            float_to_uint(out.kp, Traits::KP_MAX, Traits::KP_MIN, 12);
        const uint16_t kd =
            float_to_uint(out.kd, Traits::KD_MAX, Traits::KD_MIN, 12);

        frame.data[0] = pos >> 8;
        frame.data[1] = pos & 0xFF;
        frame.data[2] = vel >> 4;
        frame.data[3] = ((vel & 0xF) << 4) | (kp >> 8);
        frame.data[4] = kp & 0xFF;
        frame.data[5] = kd >> 4;
        frame.data[6] = ((kd & 0xF) << 4) | (tor >> 8);
        frame.data[7] = tor & 0xFF;

        return this->m_driver.send(frame);
    }

    tl::expected<void, Error> sendPosVelFrame(const DmControlOutput &out) {
        Can::CanFrame frame{};
        frame.id = m_canId + 0x100;
        frame.dlc = 8;

        const auto *pbuf = reinterpret_cast<const uint8_t *>(&out.position);
        const auto *vbuf = reinterpret_cast<const uint8_t *>(&out.angular);

        std::copy_n(pbuf, 4, frame.data);
        std::copy_n(vbuf, 4, frame.data + 4);

        return this->m_driver.send(frame);
    }

    tl::expected<void, Error> sendVelFrame(const DmControlOutput &out) {
        Can::CanFrame frame{};
        frame.id = m_canId + 0x200;
        frame.dlc = 4;

        const auto *vbuf = reinterpret_cast<const uint8_t *>(&out.angular);
        std::copy_n(vbuf, 4, frame.data);

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

    void onFeedback(Can::CanFrame &&frame) {
        this->m_buffer.push(typename Traits::StatusType(frame));
    }

    uint16_t m_masterId{}, m_canId{};
};

} // namespace OneMotor::Motor::DM

#endif // ONE_MOTOR_DM_DMMOTOR_HPP_
