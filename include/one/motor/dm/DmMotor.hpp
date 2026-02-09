#ifndef ONE_MOTOR_DM_DMMOTOR_HPP_
#define ONE_MOTOR_DM_DMMOTOR_HPP_

#include "DmFrame.hpp"
#include "DmModels.hpp"
#include <one/motor/IMotor.hpp>

#include "DmFrame.hpp"
#include "DmParam.hpp"
#include "one/utils/DoubleBuffer.hpp"

#include <cstdint>
#include <cstring>
#include <one/can/CanDriver.hpp>
#include <one/can/CanFrame.hpp>
#include <one/motor/Error.hpp>
#include <one/utils/Panic.hpp>
#include <type_traits>

namespace one::motor::dm {
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

template <typename Model> class DmMotor : public IMotor {
  public:
    DmMotor() = default;

    DmMotor(can::CanDriver &driver, const Param &param) {
        if (auto result = init(driver, param); !result) {
            panic(result.error().message);
        }
    }

    tl::expected<void, Error> init(can::CanDriver &driver, const Param &param) {
        if (const auto base_result = IMotor::init(driver); !base_result) {
            return base_result;
        }
        m_param = param;
        std::visit(
            [this]<typename T>(const T &) {
                if constexpr (std::same_as<T, MITMode>) {
                    m_pending_frame.id = m_param.can_id;
                    m_pending_frame.dlc = 8;
                    m_frame_size = 8;
                    m_compute_func = [this] { this->computeMIT(); };
                } else if constexpr (std::same_as<T, PosAngMode>) {
                    m_pending_frame.id = m_param.can_id + 0x100;
                    m_pending_frame.dlc = 8;
                    m_frame_size = 8;
                    m_compute_func = [this] { this->computePosAng(); };
                } else {
                    m_pending_frame.id = m_param.can_id + 0x200;
                    m_pending_frame.dlc = 4;
                    m_frame_size = 4;
                    m_compute_func = [this] { this->computeAng(); };
                }
            },
            m_param.mode);
        auto driver_ptr = this->driver();
        if (!driver_ptr) {
            return tl::make_unexpected(Error{ErrorCode::MotorNotInitialized,
                                             "Motor has not been initialized"});
        }
        auto result = driver_ptr->open().and_then([this, &driver_ptr] {
            return driver_ptr->registerCallback(
                {m_param.master_id},
                [this](const can::CanFrame frame) { this->onFeedback(frame); });
        });

        if (!result) {
            this->resetInitialization();
        }
        return result;
    }

    tl::expected<void, Error> setZeroPosition() {
        return sendControlFrame(detail::SETZERO_FRAME_DATA);
    }

    tl::expected<void, Error> clearError() {
        return sendControlFrame(detail::CLEAN_ERROR_DATA);
    }

    tl::expected<void, Error> sendRefreshStatus() {
        can::CanFrame frame{};
        frame.dlc = 4;
        frame.id = 0x7FF;
        auto &data = frame.data;
        const auto *ibuf = reinterpret_cast<const uint8_t *>(&m_param.can_id);
        data[0] = *(ibuf);
        data[1] = *(ibuf + 1);
        data[2] = 0xCC;
        data[3] = 0x00;
        return this->driver()->send(frame);
    }

    tl::expected<void, Error> enable() final {
        return sendControlFrame(detail::ENABLE_FRAME_DATA);
    }

    tl::expected<void, Error> disable() final {
        return sendControlFrame(detail::DISABLE_FRAME_DATA);
    }

    MotorStatus getStatus() {
        return MotorStatus::fromPlain(m_buffer.readCopy());
    }

    tl::expected<AnyStatus, Error> getStatusVariant() final {
        return MotorStatus::fromPlain(this->m_buffer.readCopy());
    }
    tl::expected<AnyPlainStatus, Error> getPlainStatusVariant() final {
        return this->m_buffer.readCopy();
    }

    void setAngRef(const float ref) noexcept override {
        m_ang_ref = ref;
        (void)update();
    };
    void setAngUnitRef(const units::AngularVelocity &ref) noexcept override {
        m_ang_ref = ref.force_numerical_value_in(units::radian / units::second);
        (void)update();
    };

    void setPosRef(const float ref) noexcept override { m_pos_ref = ref; };
    void setPosUnitRef(const units::Angle &ref) noexcept override {
        m_pos_ref = ref.force_numerical_value_in(units::radian);
        (void)update();
    };

    void setTorRef(const float ref) noexcept override {
        m_tor_ref = ref;
        (void)update();
    };
    void setTorUnitRef(const units::Torque &ref) noexcept override {
        m_tor_ref = ref.force_numerical_value_in(units::newton * units::metre);
        (void)update();
    };

    void setRefs(const float pos_ref, const float ang_ref,
                 const float tor_ref) noexcept override {
        m_pos_ref = pos_ref;
        m_ang_ref = ang_ref;
        m_tor_ref = tor_ref;
        (void)update();
    };

    void setUnitRefs(const units::Angle &pos_ref,
                     const units::AngularVelocity &ang_ref,
                     const units::Torque &tor_ref) noexcept override {
        m_ang_ref = ang_ref.force_numerical_value_in(units::literals::rad /
                                                     units::literals::s);
        m_pos_ref = pos_ref.force_numerical_value_in(units::literals::rad);
        m_tor_ref =
            tor_ref.force_numerical_value_in(units::newton * units::metre);
        (void)update();
    };

    void setParam(const Param &param) noexcept { m_param = param; }

  private:
    tl::expected<void, Error> sendControlFrame(const uint8_t *data) {
        memcpy(&m_pending_frame.data, data, 8);
        return this->driver()->send(m_pending_frame);
    };

    tl::expected<void, Error> update() {
        m_compute_func();
        std::memcpy(m_pending_frame.data, m_data, m_frame_size);
        return driver()->send(m_pending_frame);
    }

    void onFeedback(can::CanFrame frame) {
        this->m_buffer.push(MotorStatusPlain(frame, TypeTag<Model>{}));
    }

    void computeMIT() {
        const uint16_t pos =
            float_to_uint<Model::P_MIN, Model::P_MAX>(m_pos_ref, 16);
        const uint16_t vel =
            float_to_uint<Model::V_MIN, Model::V_MAX>(m_ang_ref, 12);
        const uint16_t tor =
            float_to_uint<Model::T_MIN, Model::T_MAX>(m_tor_ref, 12);
        const auto [Kp, Kd] = std::get<MITMode>(m_param.mode);
        const uint16_t kp = float_to_uint<Model::KP_MIN, Model::KP_MAX>(Kp, 12);
        const uint16_t kd = float_to_uint<Model::KD_MIN, Model::KD_MAX>(Kd, 12);
        m_data[0] = static_cast<uint8_t>(pos >> 8);
        m_data[1] = static_cast<uint8_t>(pos & 0xFF);
        m_data[2] = static_cast<uint8_t>(vel >> 4);
        m_data[3] = static_cast<uint8_t>(((vel & 0xF) << 4) | (kp >> 8));
        m_data[4] = static_cast<uint8_t>(kp & 0xFF);
        m_data[5] = static_cast<uint8_t>(kd >> 4);
        m_data[6] = static_cast<uint8_t>(((kd & 0xF) << 4) | (tor >> 8));
        m_data[7] = static_cast<uint8_t>(tor & 0xFF);
    }

    void computePosAng() {
        const auto *pbuf = reinterpret_cast<const uint8_t *>(&m_pos_ref);
        const auto *vbuf = reinterpret_cast<const uint8_t *>(&m_ang_ref);
        std::copy_n(pbuf, 4, m_data);
        std::copy_n(vbuf, 4, m_data + 4);
    }

    void computeAng() {
        const auto *vbuf = reinterpret_cast<const uint8_t *>(&m_ang_ref);
        std::copy_n(vbuf, 4, m_data);
    }

    Param m_param;
    std::function<void()> m_compute_func{};
    DoubleBuffer<MotorStatusPlain> m_buffer{};
    can::CanFrame m_pending_frame{};
    uint8_t m_frame_size{};
    float m_pos_ref{};
    float m_ang_ref{};
    float m_tor_ref{};
    uint8_t m_data[8]{};
};

using J4310 = DmMotor<J4310Model>;
using J4340 = DmMotor<J4340Model>;
using J8009 = DmMotor<J8009Model>;
using J10010L = DmMotor<J10010LModel>;

} // namespace one::motor::dm

#endif // ONE_MOTOR_DM_DMMOTOR_HPP_
