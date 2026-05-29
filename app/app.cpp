
#include "app.h"

#include "math.h"
#include "tim.h"
//

#include "../maidui3_hal/Control/PID/Proportional-Integral-Derivative.hpp"
#include "../maidui3_hal/Drivers/FDCAN/mXCAN.hpp"
#include "driver_fdcan.hpp"
#include "gn10_can/core/fdcan_bus.hpp"
#include "gn10_can/devices/esc_hub_config.hpp"
#include "gn10_can/devices/esc_hub_server.hpp"

gn10_can::drivers::DriverSTM32FDCAN main_bus_fdcan(&hfdcan1);
gn10_can::FDCANBus main_fdcan_bus(main_bus_fdcan);
gn10_can::devices::ESCHubServer main_bus(main_fdcan_bus, 0);
gn10_can::devices::ESCHubConfig hub_config;

#define maidui3_xcan maidui3_hal::Drivers::XCAN
#define C620_bace_id 0x200

maidui3_xcan::xcan esc_bus(&hfdcan2, maidui3_xcan::fifo::FIFO1, maidui3_xcan::id_filter_type::mask_four_id, 0, 1);
maidui3_xcan::hxcan_frame c620_s;
maidui3_xcan::hxcan_frame c620_r[4];

union c620_send_data_frame {
    uint8_t data[8] = {0};
    int16_t value[4];  // -16384 ~ 16384
};

union c620_receive_data_frame {
    uint8_t data[8] = {0};
    struct {
        uint16_t rote_angle; /*角度*/
        uint16_t rpm;        /*速度*/
        uint8_t temperature; /*温度*/
        uint8_t unused;
    } value;
};

c620_send_data_frame c620_send;
c620_receive_data_frame c620_receive[4];

struct c620_speed_box {
    uint16_t c620_value = {0};
    float rad_p_s       = {0};
};

int16_t c620_current_to_current(int16_t current)
{
    return (int16_t)((current & 0xFF) << 8) | ((current & 0xFF00) >> 8);
}

// CAN

maidui3_hal::Control::PID::Proportional_Integral_Derivative PID[4];

// PID

float target_value[4];

// public value
void Error();

void setup()
{
    HAL_Delay(10);
    HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);

    c620_s.id_     = C620_bace_id;
    c620_s.len_    = 0x08;
    c620_s.data_p_ = c620_send.data;

    for (uint8_t i = 0; i < 4; i++) {
        c620_r[i].id_     = 0x00;
        c620_r[i].len_    = 0x00;
        c620_r[i].data_p_ = c620_receive[i].data;
    }

    /*領域の定義*/

    esc_bus.set_Id(C620_bace_id + 1);
    esc_bus.set_Id(C620_bace_id + 2);
    esc_bus.set_Id(C620_bace_id + 3);
    esc_bus.set_Id(C620_bace_id + 4);

    esc_bus.set_Id_mask(0x7FF);

    if (esc_bus.init()) {
        HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
    }

    c620_send.value[0] = 0x00;
    c620_send.value[1] = 0x00;
    c620_send.value[2] = 0x00;
    c620_send.value[3] = 0x00;

    if (esc_bus.SendMessage(&c620_s)) {
        HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
        /*送信できてない*/
    } /*全ESC停止*/

    HAL_Delay(10);

    /*CANの初期化*/

    // PID[0].set_gain(0.1, 0.1, 0.1); /*各モーター用のPID*/
    // PID[1].set_gain(0.1, 0.1, 0.1); /*各モーター用のPID*/
    // PID[2].set_gain(0.1, 0.1, 0.1); /*各モーター用のPID*/
    // PID[3].set_gain(0.1, 0.1, 0.1); /*各モーター用のPID*/

    PID[0].set_control_cycle(1000); /*PIDの制御周期*/
    PID[1].set_control_cycle(1000); /*PIDの制御周期*/
    PID[2].set_control_cycle(1000); /*PIDの制御周期*/
    PID[3].set_control_cycle(1000); /*PIDの制御周期*/

    HAL_TIM_Base_Start_IT(&htim6);

    HAL_Delay(50);

    /*キャリブレーション開始*/

    /*モーター停止*/

    c620_send.value[0] = 0;
    c620_send.value[1] = 0;
    c620_send.value[2] = 0;
    c620_send.value[3] = 0;

    esc_bus.SendMessage(&c620_s);

    /*キャリブレーション終了*/
    HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);

    /*ループ開始*/
    HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET);
}

void loop()
{
    if (main_fdcan.buffer.nvic_.Rx_Callback) {
        if (main_bus.get_gain(hub_config)) {
            PID[0].set_gain(hub_config.kp, hub_config.ki, hub_config.kd); /*各モーター用のPID*/
            PID[1].set_gain(hub_config.kp, hub_config.ki, hub_config.kd); /*各モーター用のPID*/
            PID[2].set_gain(hub_config.kp, hub_config.ki, hub_config.kd); /*各モーター用のPID*/
            PID[3].set_gain(hub_config.kp, hub_config.ki, hub_config.kd); /*各モーター用のPID*/
        }

        if (main_bus.get_angular_velocities(target_value)) {
            PID[0].set_target(target_value[0]);
            PID[1].set_target(target_value[1]);
            PID[2].set_target(target_value[2]);
            PID[3].set_target(target_value[3]);
        }
    }

#define FF_Active_rad          2
#define set_FF_Active_Position 2 * M_PI* FF_Active_rad

    if (esc_bus.buffer.nvic_.Rx_Callback) {
        if (esc_bus.buffer.nvic_.Id[0]) {
            esc_bus.GetMessage(&c620_r[0]);
            c620_send.value[0] = 0;

            if ((target_value[0] - (c620_receive[0].value.rpm * 2 * M_PI)) >= set_FF_Active_Position) {
                c620_send.value[0] += c620_current_to_current(hub_config.ff * (target_value[0] - (c620_receive[0].value.rpm * 2 * M_PI)));
            }

            c620_send.value[0] += c620_current_to_current(PID[0].PID(c620_receive[0].value.rpm * 2 * M_PI));

            esc_bus.SendMessage(&c620_s);
        }

        if (esc_bus.buffer.nvic_.Id[1]) {
            esc_bus.GetMessage(&c620_r[1]);
            c620_send.value[1] = 0;

            if ((target_value[1] - (c620_receive[1].value.rpm * 2 * M_PI)) >= set_FF_Active_Position) {
                c620_send.value[1] += c620_current_to_current(hub_config.ff * (target_value[1] - (c620_receive[1].value.rpm * 2 * M_PI)));
            }

            c620_send.value[1] += c620_current_to_current(PID[1].PID(c620_receive[1].value.rpm * 2 * M_PI));

            esc_bus.SendMessage(&c620_s);
        }

        if (esc_bus.buffer.nvic_.Id[2]) {
            esc_bus.GetMessage(&c620_r[2]);
            c620_send.value[2] = 0;

            if ((target_value[2] - (c620_receive[2].value.rpm * 2 * M_PI)) >= set_FF_Active_Position) {
                c620_send.value[2] += c620_current_to_current(hub_config.ff * (target_value[2] - (c620_receive[2].value.rpm * 2 * M_PI)));
            }

            c620_send.value[2] += c620_current_to_current(PID[2].PID(c620_receive[2].value.rpm * 2 * M_PI));

            esc_bus.SendMessage(&c620_s);
        }

        if (esc_bus.buffer.nvic_.Id[3]) {
            esc_bus.GetMessage(&c620_r[3]);
            c620_send.value[3] = 0;

            if ((target_value[3] - (c620_receive[3].value.rpm * 2 * M_PI)) >= set_FF_Active_Position) {
                c620_send.value[3] += c620_current_to_current(hub_config.ff * (target_value[3] - (c620_receive[3].value.rpm * 2 * M_PI)));
            }

            c620_send.value[3] += c620_current_to_current(PID[3].PID(c620_receive[3].value.rpm * 2 * M_PI));

            esc_bus.SendMessage(&c620_s);
        }
    }
}

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if (htim == &htim6) {
        esc_bus.SendMessage_for_timer_loop(&c620_s);
    }
}

void Error()
{
    c620_send.value[0] = 0x00;
    c620_send.value[1] = 0x00;
    c620_send.value[2] = 0x00;
    c620_send.value[3] = 0x00;
    while (1) {
    };
}
