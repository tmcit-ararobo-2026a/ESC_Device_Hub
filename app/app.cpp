
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
gn10_can::devices::ESCHubServer esc_hub_server(main_fdcan_bus, 0);
gn10_can::devices::ESCHubConfig hub_config;

#define maidui3_xcan maidui3_hal::Drivers::XCAN
#define C620_bace_id 0x200

maidui3_xcan::xcan esc_bus(&hfdcan2, maidui3_xcan::fifo::FIFO1, maidui3_xcan::id_filter_type::mask_four_id, 0, 0);
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
        uint16_t torque;     /*トルク*/
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
    return static_cast<int16_t>(((current & 0xFF) << 8) | ((current & 0xFF00) >> 8));
}

int16_t c620_rpm_to_rpm(uint16_t rpm)
{
    return static_cast<int16_t>(((rpm & 0xFF) << 8) | ((rpm & 0xFF00) >> 8));
}

// CAN

maidui3_hal::Control::PID::Proportional_Integral_Derivative PID[4];

// PID

float target_value[4];

// public value
bool timer_1kHz;

// test

void Error();

void setup()
{
    HAL_Delay(10);
    HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);

    for (uint8_t i = 0; i < 30; i++) {
        HAL_Delay(100);
        HAL_GPIO_TogglePin(LED_2_GPIO_Port, LED_2_Pin);
    } /*起動時確認用トグル3秒*/

    c620_s.id_     = C620_bace_id;   /*送信id               絶対定義しないといけない*/
    c620_s.len_    = 0x08;           /*送信サイズ            絶対定義しないといけない*/
    c620_s.data_p_ = c620_send.data; /*送信する変数のポインタ 絶対定義しないといけない*/

    for (uint8_t i = 0; i < 4; i++) {
        c620_r[i].id_     = 0x00;                 /*受信idが入る変数の初期化*/
        c620_r[i].len_    = 0x00;                 /*受信したデータのサイズを入れる変数の初期化*/
        c620_r[i].data_p_ = c620_receive[i].data; /*受信した値を入れるポインタを定義            絶対定義しないといけない*/
    } /*受信idは使わなくてもいい*/

    /*領域の定義*/

    esc_bus.set_Id(C620_bace_id + 1); /*受信したいidを定義する*/  // Index 0
    esc_bus.set_Id(C620_bace_id + 2); /*受信したいidを定義する*/  // Index 1
    esc_bus.set_Id(C620_bace_id + 3); /*受信したいidを定義する*/  // Index 2
    esc_bus.set_Id(C620_bace_id + 4); /*受信したいidを定義する*/  // Index 3
    /*idは最大4つまで*/

    esc_bus.set_Id_mask(0x7FF); /*idに対してどのようなマスクをかけるのか*/
    /*実際に使われるidは、set_Id & set_Id_mask の論理値で出る*/

    if (esc_bus.init()) { /*初期化*/
        HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
    }

    c620_send.value[0] = 0x00;
    c620_send.value[1] = 0x00;
    c620_send.value[2] = 0x00;
    c620_send.value[3] = 0x00;

    if (esc_bus.SendMessage(&c620_s)) { /*送信*/
        HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
        /*送信できてない*/
    } /*全ESC停止*/

    HAL_Delay(10);

    /*CANの初期化*/

    PID[0].set_max_sum_deviation(4000000000000.0f);
    PID[0].set_max_sum_deviation(4000000000000.0f);
    PID[0].set_max_sum_deviation(4000000000000.0f);
    PID[0].set_max_sum_deviation(4000000000000.0f);

    hub_config.ff = 300.0f;
    PID[0].set_gain(6000.0f, 50.0f, 3.0f); /*各モーター用のPID*/
    PID[1].set_gain(6000.0f, 50.0f, 3.0f); /*各モーター用のPID*/
    PID[2].set_gain(6000.0f, 50.0f, 3.0f); /*各モーター用のPID*/
    PID[3].set_gain(6000.0f, 50.0f, 3.0f); /*各モーター用のPID*/

    PID[0].set_control_cycle(1000); /*PIDの制御周期 Hz*/
    PID[1].set_control_cycle(1000); /*PIDの制御周期 Hz*/
    PID[2].set_control_cycle(1000); /*PIDの制御周期 Hz*/
    PID[3].set_control_cycle(1000); /*PIDの制御周期 Hz*/

    PID[0].reset_deviation(); /*PID内部の積分を初期化*/
    PID[1].reset_deviation(); /*PID内部の積分を初期化*/
    PID[2].reset_deviation(); /*PID内部の積分を初期化*/
    PID[3].reset_deviation(); /*PID内部の積分を初期化*/

    HAL_TIM_Base_Start_IT(&htim6); /*1kHzの周期*/

    HAL_Delay(100);

    /*モーター停止*/
    c620_send.value[0] = 0;
    c620_send.value[1] = 0;
    c620_send.value[2] = 0;
    c620_send.value[3] = 0;

    esc_bus.SendMessage(&c620_s);

    target_value[0] = 0;
    target_value[1] = 0;
    target_value[2] = 0;
    target_value[3] = 0;

    PID[0].set_target(target_value[0]);
    PID[1].set_target(target_value[1]);
    PID[2].set_target(target_value[2]);
    PID[3].set_target(target_value[3]);

    HAL_Delay(300);
    HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);

    /*ループ開始*/
    HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET);
}

void loop()
{
    if (main_fdcan.buffer.nvic_.Rx_Callback) {
        if (esc_hub_server.get_gain(hub_config)) {
            PID[0].set_gain(hub_config.kp, hub_config.ki, hub_config.kd); /*各モーター用のPID*/
            PID[1].set_gain(hub_config.kp, hub_config.ki, hub_config.kd); /*各モーター用のPID*/
            PID[2].set_gain(hub_config.kp, hub_config.ki, hub_config.kd); /*各モーター用のPID*/
            PID[3].set_gain(hub_config.kp, hub_config.ki, hub_config.kd); /*各モーター用のPID*/
        }

        if (esc_hub_server.get_angular_velocities(target_value)) {
            PID[0].set_target(target_value[0]);
            PID[1].set_target(target_value[1]);
            PID[2].set_target(target_value[2]);
            PID[3].set_target(target_value[3]);
        }
        main_fdcan.buffer.nvic_.Rx_Callback = 0;
    }

#define FF_Active_rad          0.5f
#define set_FF_Active_Position 2.0f * M_PI* FF_Active_rad

    if (esc_bus.buffer.nvic_.Rx_Callback) { /*バス全体の割り込みフラグ*/

        if (esc_bus.buffer.nvic_.Id[0]) {      /*Index 0 の割り込みフラグ*/
            esc_bus.GetMessage(&c620_r[0], 0); /*Index 0 の受信*/

            static float send_value_0;
            static float rad_p_s_0;

            rad_p_s_0 = (float)c620_rpm_to_rpm(c620_receive[0].value.rpm) / 19.0f / 60.0f * 2.0f * (float)M_PI;

            if (abs(target_value[0] - rad_p_s_0) >= set_FF_Active_Position) {
                send_value_0 = (int16_t)(hub_config.ff * (target_value[0] - rad_p_s_0));
            }

            send_value_0 += PID[0].PID(rad_p_s_0);

            c620_send.value[0] = c620_current_to_current((int16_t)(send_value_0 > 32767 ? 32767 : send_value_0 < -32768 ? -32768 : send_value_0));

            esc_bus.SendMessage(&c620_s);

            esc_bus.buffer.nvic_.Id[0] = 0;
        }

        if (esc_bus.buffer.nvic_.Id[1]) {
            esc_bus.GetMessage(&c620_r[1], 1);

            static float send_value_1;
            static float rad_p_s_1;

            rad_p_s_1 = (float)c620_rpm_to_rpm(c620_receive[1].value.rpm) / 19.0f / 60.0f * 2.0f * (float)M_PI;

            if (abs(target_value[1] - rad_p_s_1) >= set_FF_Active_Position) {
                send_value_1 = (int16_t)(hub_config.ff * (target_value[1] - rad_p_s_1));
            }

            send_value_1 += PID[1].PID(rad_p_s_1);

            c620_send.value[1] = c620_current_to_current((int16_t)(send_value_1 > 32767 ? 32767 : send_value_1 < -32768 ? -32768 : send_value_1));

            esc_bus.SendMessage(&c620_s);

            esc_bus.buffer.nvic_.Id[1] = 0;
        }

        if (esc_bus.buffer.nvic_.Id[2]) {
            esc_bus.GetMessage(&c620_r[2], 2);

            static float send_value_2;
            static float rad_p_s_2;

            rad_p_s_2 = (float)c620_rpm_to_rpm(c620_receive[2].value.rpm) / 19.0f / 60.0f * 2.0f * (float)M_PI;

            if (abs(target_value[2] - rad_p_s_2) >= set_FF_Active_Position) {
                send_value_2 = (int16_t)(hub_config.ff * (target_value[2] - rad_p_s_2));
            }

            send_value_2 += PID[2].PID(rad_p_s_2);

            c620_send.value[2] = c620_current_to_current((int16_t)(send_value_2 > 32767 ? 32767 : send_value_2 < -32768 ? -32768 : send_value_2));

            esc_bus.SendMessage(&c620_s);

            esc_bus.buffer.nvic_.Id[2] = 0;
        }

        if (esc_bus.buffer.nvic_.Id[3]) {
            esc_bus.GetMessage(&c620_r[3], 3);
            c620_send.value[3] = 0;

            static float send_value_3;
            static float rad_p_s_3;

            rad_p_s_3 = (float)c620_rpm_to_rpm(c620_receive[3].value.rpm) / 19.0f / 60.0f * 2.0f * (float)M_PI;

            if (abs(target_value[3] - rad_p_s_3) >= set_FF_Active_Position) {
                send_value_3 = (int16_t)(hub_config.ff * (target_value[3] - rad_p_s_3));
            }

            send_value_3 += PID[3].PID(rad_p_s_3);

            c620_send.value[3] = c620_current_to_current((int16_t)(send_value_3 > 32767 ? 32767 : send_value_3 < -32768 ? -32768 : send_value_3));

            esc_bus.SendMessage(&c620_s);

            esc_bus.buffer.nvic_.Id[3] = 0;
        }

        esc_bus.buffer.nvic_.Rx_Callback = 0;
    }

    if (timer_1kHz) {
        esc_bus.SendMessage_for_timer_loop(&c620_s);
        timer_1kHz = 0;
    }
}

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if (htim == &htim6) {
        timer_1kHz = 1;
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
