#include <cstring>

#include "one-motor/can/CanFrame.hpp"

namespace OneMotor::Can
{
    CanFrame::operator can_frame() const
    {
        can_frame frame{.can_id = id, .len = dlc};
        memcpy(frame.data, data, ONE_MOTOR_CAN_MAX_DLEN);
        return frame;
    }

    CanFrame::CanFrame(const can_frame& frame): flags(0)
    {
        id = frame.can_id;
        dlc = frame.len;
        memcpy(data, frame.data, ONE_MOTOR_CAN_MAX_DLEN);
    }
}
