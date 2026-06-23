#include "esc_control_hub/c6x_can.hpp"

#include <cstring>

#include "gn10_can/utils/can_converter.hpp"

namespace c6x0_can {
void C6XCAN::update()
{
    gn10_can::CANFrame frame;
    if (!can_driver_.receive(frame)) return;
    if ((frame.id & 0xFF0) != 0x200) return;

    uint8_t motor_number = (frame.id & 0x0F) - 1;
    if (motor_number > 7) return;  // ガード

    C620Feedback received_feedback;
    memcpy(&received_feedback, &frame.data, 8);

    // c620はビックエンディアンなのでリトルエンディアンに変換
    feedback_[motor_number].angle       = __builtin_bswap16(received_feedback.angle);
    feedback_[motor_number].speed_rpm   = __builtin_bswap16(received_feedback.speed_rpm);
    feedback_[motor_number].current     = __builtin_bswap16(received_feedback.current);
    feedback_[motor_number].temperature = received_feedback.temperature;
}

bool C6XCAN::set_currents_1_3(int16_t currents[4])
{
    std::array<uint8_t, 8> data;
    uint16_t current_data[4];
    for (uint8_t i = 0; i < 4; i++) {
        current_data[i] = static_cast<uint16_t>(currents[i]);
        current_data[i] = __builtin_bswap16(current_data[i]);
        gn10_can::converter::pack(data, i * sizeof(uint16_t), current_data[i]);
    }
    gn10_can::CANFrame frame;
    frame.data = data;
    frame.dlc  = 8;
    frame.id   = SEND_CANID_0_3;
    return can_driver_.send(frame);
}

bool C6XCAN::set_currents_4_7(int16_t currents[4])
{
    std::array<uint8_t, 8> data;
    uint16_t current_data[4];
    for (uint8_t i = 0; i < 4; i++) {
        current_data[i] = static_cast<uint16_t>(currents[i]);
        current_data[i] = __builtin_bswap16(current_data[i]);
        gn10_can::converter::pack(data, i * sizeof(uint16_t), current_data[i]);
    }
    gn10_can::CANFrame frame;
    frame.data = data;
    frame.dlc  = 8;
    frame.id   = SEND_CANID_4_7;
    return can_driver_.send(frame);
}

uint16_t C6XCAN::get_feedback_angle(const uint8_t motor_id) const
{
    if (motor_id > 7) return 0;
    return feedback_[motor_id].angle;
}

int16_t C6XCAN::get_feedback_speed(const uint8_t motor_id) const
{
    if (motor_id > 7) return 0;
    return feedback_[motor_id].speed_rpm;
}

int16_t C6XCAN::get_feedback_current(const uint8_t motor_id) const
{
    if (motor_id > 7) return 0;
    return feedback_[motor_id].current;
}

uint8_t C6XCAN::get_feedback_temperature(const uint8_t motor_id) const
{
    if (motor_id > 7) return 0;
    return feedback_[motor_id].temperature;
}
}  // namespace c6x0_can