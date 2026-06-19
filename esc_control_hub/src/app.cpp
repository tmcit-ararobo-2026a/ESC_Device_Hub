#include "esc_control_hub/app.hpp"
// std
#include <cmath>
// STM32 HAL
#include "fdcan.h"
// gn10-can
#include "gn10_can/core/can_bus.hpp"
#include "gn10_can/devices/esc_hub_server.hpp"
// esc-control-hub
#include "esc_control_hub/can_driver.hpp"
#include "esc_control_hub/fdcan_driver.hpp"
#include "esc_control_hub/pid.hpp"
// others

namespace {

constexpr uint32_t k_heartbeat_toggle_interval_ms = 500;
constexpr uint32_t k_target_last_update_time_ms   = 500;

uint32_t heartbeat_last_toggle_time_ms = 0;
uint32_t target_last_update_time_ms    = 0;
// configuration
gn10_motor::PIDConfig<float> pid_config[4];
// Calc
gn10_motor::PID<float> pid[4]{
    gn10_motor::PID(pid_config[0]), gn10_motor::PID(pid_config[1]), gn10_motor::PID(pid_config[2]), gn10_motor::PID(pid_config[3])
};

gn10_can::devices::MotorConfig motor_configres[4];
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
// CAN Drivers
gn10_can::drivers::FDCANDriver fdcan1_driver(&hfdcan1);
gn10_can::drivers::CANDriver can2_driver(&hfdcan2);
// CAN Bus
gn10_can::FDCANBus fdcan1_bus(fdcan1_driver);
// CAN Devices
gn10_can::devices::ESCHubServer server(fdcan1_bus, 1);

}  // namespace

/**
 * @brief Initialize CAN and mainboard application state.
 */
void setup()
{
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
    // Update targets
    const uint32_t now_ms = HAL_GetTick();
    if (server.get_angular_velocities(targets)) {
        target_last_update_time_ms = now_ms;
    }

    // Update parameters by client and calculate PID
    for (uint8_t i = 0; i < 4; i++) {
        // Update configuration
        if (server.get_init(i, motor_configres[i])) {
        }
        // Update gains
        float ff_gain;
        if (server.get_gains(i, pid_config[i].kp, pid_config[i].ki, pid_config[i].kd, ff_gain)) {
            pid[i].set_config(pid_config[i]);
        }
        // calculate PID
        currents[i] = pid[i].update(targets[i], 0.0f, 0.001f);
    }
    // Safety guard
    if ((now_ms - target_last_update_time_ms) < k_target_last_update_time_ms) {
    }

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
    }
}
}