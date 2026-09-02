#include "esc_control_hub/c6x_can.hpp"

#include <algorithm>
#include <climits>
#include <cstring>

#include "gn10_can/utils/can_converter.hpp"

namespace c6x0_can {

void C6XCAN::set_motor_type(int motor_index, gn10_can::devices::MotorType motor_type)
{
    if (motor_index < 0 || motor_index >= 8) return;
    switch (motor_type) {
        case gn10_can::devices::MotorType::C610:
            max_currents_abs_[motor_index]           = C610_MAX_CURRENT_ABS;
            current_to_data_conversion_[motor_index] = C610_CURRENT_CONVERSION;
            break;

        case gn10_can::devices::MotorType::C620:
            max_currents_abs_[motor_index]           = C620_MAX_CURRENT_ABS;
            current_to_data_conversion_[motor_index] = C620_CURRENT_CONVERSION;
            break;

        default:
            break;
    }
}
void C6XCAN::update()
{
    gn10_can::CANFrame frame;
    if (!can_driver_.receive(frame)) return;
    // C610 / C620 のフィードバック CAN ID (0x201 〜 0x208) のみ通過
    if (frame.id < 0x201 || frame.id > 0x208) return;
    uint8_t motor_number = static_cast<uint8_t>(frame.id - 0x201);

    C620Feedback received_feedback;
    memcpy(&received_feedback, frame.data.data(), 8);

    // c620はビックエンディアンなのでリトルエンディアンに変換
    feedback_[motor_number].angle       = __builtin_bswap16(received_feedback.angle);
    feedback_[motor_number].speed_rpm   = __builtin_bswap16(received_feedback.speed_rpm);
    feedback_[motor_number].current     = __builtin_bswap16(received_feedback.current);
    feedback_[motor_number].temperature = received_feedback.temperature;
}

bool C6XCAN::set_currents_1_4(const std::array<float, 4>& currents)
{
    std::array<float, 4> scaled_currents{};
    std::array<uint8_t, 8> data{};
    std::array<uint16_t, 4> packed_currents{};
    for (uint8_t i = 0; i < 4; i++) {
        scaled_currents[i] = std::clamp((currents[i] * current_to_data_conversion_[i]), -max_currents_abs_[i], max_currents_abs_[i]);
        int16_t current    = (int16_t)std::clamp(scaled_currents[i], static_cast<float>(INT16_MIN), static_cast<float>(INT16_MAX));
        packed_currents[i] = static_cast<uint16_t>(current);
        packed_currents[i] = __builtin_bswap16(packed_currents[i]);
        gn10_can::converter::pack(data, i * sizeof(uint16_t), packed_currents[i]);
    }
    gn10_can::CANFrame frame;
    frame.data = data;
    frame.dlc  = 8;
    frame.id   = SEND_CANID_0_3;
    return can_driver_.send(frame);
}

bool C6XCAN::set_currents_5_8(const std::array<float, 4>& currents)
{
    std::array<float, 4> scaled_currents{};
    std::array<uint8_t, 8> data{};
    std::array<uint16_t, 4> packed_currents{};
    for (uint8_t i = 0; i < 4; i++) {
        scaled_currents[i] = std::clamp((currents[i] * current_to_data_conversion_[i + 4]), -max_currents_abs_[i + 4], max_currents_abs_[i + 4]);
        int16_t current    = (int16_t)std::clamp(scaled_currents[i], static_cast<float>(INT16_MIN), static_cast<float>(INT16_MAX));
        packed_currents[i] = static_cast<uint16_t>(current);
        packed_currents[i] = __builtin_bswap16(packed_currents[i]);
        gn10_can::converter::pack(data, i * sizeof(uint16_t), packed_currents[i]);
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