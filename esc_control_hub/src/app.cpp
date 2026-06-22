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
#include "esc_control_hub/pid.hpp"
// others

namespace {

constexpr uint32_t k_heartbeat_toggle_interval_ms = 500;
constexpr uint32_t k_target_last_update_time_ms   = 500;

uint32_t heartbeat_last_toggle_time_ms = 0;
uint32_t target_last_update_time_ms    = 0;
float feedbacks[4];
// Configuration
gn10_motor::PIDConfig<float> pid_config[4];
gn10_can::devices::MotorConfig motor_configres[4];
// Calculate
gn10_motor::PID<float> pid[4]{
    gn10_motor::PID(pid_config[0]), gn10_motor::PID(pid_config[1]), gn10_motor::PID(pid_config[2]), gn10_motor::PID(pid_config[3])
};

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
    }

    // CAN initialization
    fdcan1_driver.init();
    can2_driver.init();
    // System setup
    heartbeat_last_toggle_time_ms = HAL_GetTick();
    target_last_update_time_ms    = HAL_GetTick();
}

/**
 * @brief Run one control cycle and update status heartbeat LED.
 */
void loop()
{
    float targets[4]  = {0.0f, 0.0f, 0.0f, 0.0f};
    float currents[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    // Update feedbacks
    for (uint8_t i = 0; i < 4; i++) {
        feedbacks[i] = 2 * M_PI * float(c6x0.get_feedback_speed(i)) / 60.0f;
    }
    // server.set_angular_velocity_feedbacks(feedbacks);
    // Update targets
    const uint32_t now_ms = HAL_GetTick();
    if (server != nullptr && server->get_angular_velocities(targets)) {
        target_last_update_time_ms = now_ms;
        HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET);
    }
    // Update parameters by client and calculate PID
    for (uint8_t i = 0; i < 4; i++) {
        // Update configuration
        if (server != nullptr && server->get_init(i, motor_configres[i])) {
            HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_SET);
        }
        // Update gains
        float ff_gain;
        if (server != nullptr && server->get_gains(i, pid_config[i].kp, pid_config[i].ki, pid_config[i].kd, ff_gain)) {
            pid[i].set_config(pid_config[i]);
            HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);
        }
        // calculate PID
        currents[i] = pid[i].update(targets[i], feedbacks[i], 0.001f);
    }
    // Currents to Integer
    int16_t current_data[4];
    for (uint8_t i = 0; i < 4; i++) {
        current_data[i] = int16_t(currents[i] * c6x0_can::C620_CURRENT_CONVERSION);
    }
    // Safety guard
    if ((now_ms - target_last_update_time_ms) > k_target_last_update_time_ms) {
        for (uint8_t i = 0; i < 4; i++) {
            current_data[0] = 0;
        }
    }
    // Send Currents
    c6x0.set_currents_1_3(current_data);

    // Basic System Process
    update_heartbeat_led();
    HAL_Delay(1);
}

// ---------------------------- C language's functions override ----------------------------------
extern "C" {
/**
 * @brief Receive callback for FDCAN FIFO0.
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs)
{
    if (hfdcan->Instance == hfdcan1.Instance) {
        fdcan1_bus.update();
    } else {
        c6x0.update();
    }
}
}