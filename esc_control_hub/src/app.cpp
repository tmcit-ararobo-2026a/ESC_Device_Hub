#include "esc_control_hub/app.hpp"
// std
#include <cmath>
#include <new>
// STM32 HAL
#include "fdcan.h"
// gn10-can
#include "gn10_can/core/can_bus.hpp"
#include "gn10_can/devices/esc_hub_server.hpp"
// esc-control-hub
#include "esc_control_hub/c6x_can.hpp"
#include "esc_control_hub/can_driver.hpp"
#include "esc_control_hub/fdcan_driver.hpp"
#include "esc_control_hub/incremental_encoder.hpp"
#include "esc_control_hub/pid.hpp"
// others

namespace {

constexpr float MOTOR_CONTROL_PERIOD              = 0.001f;
constexpr uint32_t k_heartbeat_toggle_interval_ms = 500;
constexpr uint32_t k_target_last_update_time_ms   = 500;
constexpr uint32_t k_send_anglar_data_interval_ms = 10;

volatile bool timer_triggered = false;
volatile float feedbacks[4];

uint32_t heartbeat_last_toggle_time_ms = 0;
uint32_t target_last_update_time_ms    = 0;
uint32_t send_anglar_data_last_time_ms = 0;

constexpr float max_current_c610 = 10.0f;
constexpr float max_current_c620 = 20.0f;
// Configuration
gn10_motor::PIDConfig<float> pid_config[4];
gn10_can::devices::MotorConfig motor_configres[4];
// Calculate
gn10_motor::PID<float> pid[4]{
    gn10_motor::PID(pid_config[0]), gn10_motor::PID(pid_config[1]), gn10_motor::PID(pid_config[2]), gn10_motor::PID(pid_config[3])
};

float targets[4]  = {0.0f, 0.0f, 0.0f, 0.0f};
float currents[4] = {0.0f, 0.0f, 0.0f, 0.0f};  // [A]

/**
 * @brief Toggle heartbeat LED at a fixed interval.
 */
void update_heartbeat_led()
{
    const uint32_t now_ms = HAL_GetTick();
    if ((now_ms - heartbeat_last_toggle_time_ms) >= k_heartbeat_toggle_interval_ms) {
        heartbeat_last_toggle_time_ms = now_ms;
        HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_1_Pin);
    }
}
/**
 * @brief Get the device id by dip switch object
 *
 * @return uint8_t device_id
 */
uint8_t get_device_id_by_dip_switch()
{
    uint8_t device_id = 0;
    if (HAL_GPIO_ReadPin(ID_2_SW_GPIO_Port, ID_2_SW_Pin)) device_id |= 0b1;
    if (HAL_GPIO_ReadPin(ID_4_SW_GPIO_Port, ID_4_SW_Pin)) device_id |= 0b10;
    return device_id;
}
// CAN Drivers
gn10_can::drivers::FDCANDriver fdcan1_driver(&hfdcan1);
gn10_can::drivers::CANDriver can2_driver(&hfdcan2);
// CAN Bus
gn10_can::FDCANBus fdcan1_bus(fdcan1_driver);
// CAN Devices
alignas(gn10_can::devices::ESCHubServer) static unsigned char server_storage[sizeof(gn10_can::devices::ESCHubServer)];
gn10_can::devices::ESCHubServer* server = nullptr;
// C620 / C610
c6x0_can::C6XCAN c6x0(can2_driver);
// Encoder
gn10_motor::IncrementalEncoder encoders[4] = {
    gn10_motor::IncrementalEncoder(4095, &htim8, TIM8),
    gn10_motor::IncrementalEncoder(4095, &htim1, TIM1),
    gn10_motor::IncrementalEncoder(4095, &htim3, TIM3),
    gn10_motor::IncrementalEncoder(4095, &htim4, TIM4)
};

void send_feedback_data(float feedback_data[4])
{
    const uint32_t now_ms = HAL_GetTick();
    if ((now_ms - send_anglar_data_last_time_ms) >= k_send_anglar_data_interval_ms) {
        send_anglar_data_last_time_ms = now_ms;
        server->set_feedbacks(feedback_data);
    }
}

void control_motors()
{
    // Update feedbacks
    for (uint8_t i = 0; i < 4; i++) {
        if (motor_configres[i].get_encoder_type() == gn10_can::devices::EncoderType::None) {
            feedbacks[i] = 2 * M_PI * float(c6x0.get_feedback_speed(i)) / 60.0f;
        }
    }
    const uint32_t now_ms = HAL_GetTick();
    if (server != nullptr && server->get_targets(targets)) {
        target_last_update_time_ms = now_ms;
    }
    // Update parameters by client and calculate PID
    for (uint8_t i = 0; i < 4; i++) {
        // Update configuration
        if (server != nullptr && server->get_init(i, motor_configres[i])) {
            HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_SET);
            float current_limit = 0.0f;
            switch (motor_configres[i].get_motor_type()) {
                case gn10_can::devices::MotorType::C610:
                    current_limit = max_current_c610;
                    break;

                case gn10_can::devices::MotorType::C620:
                    current_limit = max_current_c620;
                    break;

                default:
                    break;
            }
            pid_config[i].output_limit = current_limit;
            encoders[i].reset();
            pid[i].set_config(pid_config[i]);
            c6x0.set_moter_type(i, motor_configres[i].get_motor_type());
        }
        // Update gains
        float ff_gain;
        if (server != nullptr && server->get_gains(i, pid_config[i].kp, pid_config[i].ki, pid_config[i].kd, ff_gain)) {
            pid[i].set_config(pid_config[i]);
            HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET);
        }
        // calculate PID
        currents[i] = pid[i].update(targets[i], feedbacks[i], MOTOR_CONTROL_PERIOD);
    }
    // Safety guard
    if ((now_ms - target_last_update_time_ms) > k_target_last_update_time_ms) {
        for (uint8_t i = 0; i < 4; i++) {
            currents[i] = 0.0f;
        }
        HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
    }
    // Send Currents
    c6x0.set_currents_1_4(currents);
}

}  // namespace

/**
 * @brief Initialize CAN and mainboard application state.
 */
void setup()
{
    // Get device id
    const uint8_t device_id = get_device_id_by_dip_switch();

    server = new (server_storage) gn10_can::devices::ESCHubServer(fdcan1_bus, device_id);

    for (uint8_t i = 0; i < 4; i++) {
        pid_config[i].output_limit   = 20.0f;
        pid_config[i].integral_limit = 1.0f;
        pid_config[i].kp             = 0.0f;
        pid_config[i].ki             = 0.0f;
        pid_config[i].kd             = 0.0f;
        pid[i].set_config(pid_config[i]);
        encoders[i].hardware_init();
    }

    // CAN initialization
    fdcan1_driver.init();
    can2_driver.init();
    HAL_TIM_Base_Start_IT(&htim6);
    // System setup
    heartbeat_last_toggle_time_ms = HAL_GetTick();
    target_last_update_time_ms    = HAL_GetTick();
}

/**
 * @brief Run one control cycle and update status heartbeat LED.
 */
void loop()
{
    c6x0.update();
    fdcan1_bus.update();
    if (timer_triggered) {
        timer_triggered = false;
        control_motors();
        float feedback_data[4] = {feedbacks[0], feedbacks[1], feedbacks[2], feedbacks[3]};
        send_feedback_data(feedback_data);
        // Basic System Process
        update_heartbeat_led();
    }
}

// ---------------------------- C language's functions override ----------------------------------
extern "C" {

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if (htim->Instance == htim6.Instance) {  // 1kHz timer
        // Update feedbacks
        for (uint8_t i = 0; i < 4; i++) {
            switch (motor_configres[i].get_encoder_type()) {
                case gn10_can::devices::EncoderType::IncrementalSpeed:
                    feedbacks[i] = encoders[i].count_to_angular_velocity(encoders[i].read_and_reset_count(), MOTOR_CONTROL_PERIOD);
                    break;
                case gn10_can::devices::EncoderType::IncrementalTotal:
                    feedbacks[i] = encoders[i].accumulate_angle_rad(encoders[i].read_and_reset_count());
                    break;
                default:
                    break;
            }
        }
        timer_triggered = true;
    }
}
}