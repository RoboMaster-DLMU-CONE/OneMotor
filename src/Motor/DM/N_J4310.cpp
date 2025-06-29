#include "OneMotor/Motor/DM/J4310.hpp"
#include "OneMotor/Thread/Othread.hpp"
#include "OneMotor/Util/Panic.hpp"

static constexpr uint8_t ENABLE_FRAME_DATA[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
static constexpr uint8_t DISABLE_FRAME_DATA[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
static constexpr uint8_t SETZERO_FRAME_DATA[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
static constexpr uint8_t CLEANERROR_FRAME_DATA[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};

namespace OneMotor::Motor::DM
{
    J4310::J4310(Can::CanDriver& driver, const uint16_t canId, const uint16_t masterId): driver_(driver),
        canId_(canId),
        masterId_(masterId)
    {
        const auto result = driver_.registerCallback({masterId_}, [&](Can::CanFrame&& frame)
        {
            const auto msg = static_cast<J4310Status>(frame);
            lock_.lock();
            status_ = msg;
            lock_.unlock();
        });

        if (!result)
        {
            Util::om_panic("Register Callback Failed for DM J4310");
        }
    }

    J4310::Result J4310::enable()
    {
        Can::CanFrame frame{};
        frame.id = canId_ + 0x100;
        frame.dlc = 8;
        std::ranges::copy(ENABLE_FRAME_DATA, frame.data);

        return driver_.send(frame);
    }

    J4310::Result J4310::disable()
    {
        Can::CanFrame frame{};
        frame.id = canId_ + 0x100;
        frame.dlc = 8;
        std::ranges::copy(DISABLE_FRAME_DATA, frame.data);

        return driver_.send(frame);
    }

    J4310::Result J4310::setZeroPosition()
    {
        Can::CanFrame frame{};
        frame.id = canId_ + 0x100;
        frame.dlc = 8;
        std::ranges::copy(SETZERO_FRAME_DATA, frame.data);

        return driver_.send(frame);
    }

    J4310::Result J4310::cleanError()
    {
        Can::CanFrame frame{};
        frame.id = canId_ + 0x100;
        frame.dlc = 8;
        std::ranges::copy(CLEANERROR_FRAME_DATA, frame.data);

        return driver_.send(frame);
    }

    J4310::Result J4310::MITControl(const float position, const float velocity, const float torque, const float kp,
                                    const float kd)
    {
        Can::CanFrame frame{};
        frame.dlc = 8;
        frame.id = canId_;
        const uint16_t pos_temp = float_to_uint(position, DM_P_MAX, DM_P_MIN, 16);
        const uint16_t vel_temp = float_to_uint(velocity, DM_V_MAX, DM_V_MIN, 12);
        const uint16_t tor_temp = float_to_uint(torque, DM_T_MAX, DM_T_MIN, 12);
        const uint16_t kp_temp = float_to_uint(kp, DM_KP_MAX, DM_KP_MIN, 12);
        const uint16_t kd_temp = float_to_uint(kd, DM_KD_MAX, DM_KD_MIN, 12);

        frame.data[0] = pos_temp >> 8;
        frame.data[1] = pos_temp;
        frame.data[2] = vel_temp >> 4;
        frame.data[3] = ((vel_temp & 0xF) << 4) | (kp_temp >> 8);
        frame.data[4] = kp_temp;
        frame.data[5] = (kd_temp >> 4);
        frame.data[6] = ((kd_temp & 0xF) << 4) | (tor_temp >> 8);
        frame.data[7] = tor_temp;
        return driver_.send(frame);
    }

    J4310::Result J4310::posVelControl(const float position, const float velocity)
    {
        Can::CanFrame frame{};
        frame.dlc = 8;
        frame.id = canId_ + 0x100;
        auto& data = frame.data;
        auto* pbuf = reinterpret_cast<const uint8_t*>(&position);
        auto* vbuf = reinterpret_cast<const uint8_t*>(&velocity);
        data[0] = *pbuf;
        data[1] = *(pbuf + 1);
        data[2] = *(pbuf + 2);
        data[3] = *(pbuf + 3);
        data[4] = *vbuf;
        data[5] = *(vbuf + 1);
        data[6] = *(vbuf + 2);
        data[7] = *(vbuf + 3);

        return driver_.send(frame);
    }

    J4310::Result J4310::velControl(const float velocity)
    {
        Can::CanFrame frame{};
        frame.dlc = 4;
        frame.id = canId_ + 0x200;
        auto& data = frame.data;
        const auto* vbuf = reinterpret_cast<const uint8_t*>(&velocity);
        data[0] = *vbuf;
        data[1] = *(vbuf + 1);
        data[2] = *(vbuf + 2);
        data[3] = *(vbuf + 3);
        return driver_.send(frame);
    }

    std::expected<J4310Status, std::string> J4310::getStatus()
    {
        lock_.lock();
        J4310Status status = status_;
        lock_.unlock();
        return status;
    }
}
