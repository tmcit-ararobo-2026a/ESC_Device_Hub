
#include "app.h"

#include "math.h"
#include "tim.h"
//

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

maidui3_xcan::xcan esc_bus_can(&hfdcan2, maidui3_xcan::fifo::FIFO1, maidui3_xcan::id_filter_type::mask_four_id, 0, 1);

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

c620_speed_box c620_line_table[4][800];
c620_speed_box c620_diagonal_table[4][800];
c620_speed_box c620_rotate_table[4][800];

int16_t c620_rad(int16_t rad)
{
    return 0;
}

int16_t c620_current(int16_t current)
{
    return (int16_t)((current & 0xFF) << 8) | ((current & 0xFF00) >> 8);
}

void three_omni_cab();
void four_omni_cab();

float PID(float diff);
float P(float diff);
float I(float diff);
float D(float diff);

void Error();

void setup()
{
    HAL_Delay(10);
    HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);

    esc_bus_can.set_Id(C620_bace_id + 1);
    esc_bus_can.set_Id(C620_bace_id + 2);
    esc_bus_can.set_Id(C620_bace_id + 3);
    esc_bus_can.set_Id(C620_bace_id + 4);

    esc_bus_can.set_Id_mask(0x7FF);

    if (esc_bus_can.init()) {
        HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
    }

    c620_s.id_     = C620_bace_id;
    c620_s.len_    = 0x08;
    c620_s.data_p_ = c620_send.data;

    for (uint8_t i = 0; i < 4; i++) {
        c620_r[i].id_     = 0x00;
        c620_r[i].len_    = 0x00;
        c620_r[i].data_p_ = c620_receive[i].data;
    }

    c620_send.value[0] = 0x00;
    c620_send.value[1] = 0x00;
    c620_send.value[2] = 0x00;
    c620_send.value[3] = 0x00;

    if (esc_bus_can.SendMessage(&c620_s)) {
        HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
    } /*全ESC停止*/

    HAL_Delay(10);

    /*CANの初期化*/

    main_bus.get_gain(hub_config);

    HAL_TIM_Base_Start_IT(&htim6);

    HAL_Delay(50);

    /*キャリブレーション開始*/

    four_omni_cab();

    /*モーター停止*/

    c620_send.value[0] = 0;
    c620_send.value[1] = 0;
    c620_send.value[2] = 0;
    c620_send.value[3] = 0;

    esc_bus_can.SendMessage(&c620_s);

    /*キャリブレーション終了*/
    HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);

    /*ループ開始*/
    HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET);
}

void loop() {}

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if (htim == &htim6) {
        esc_bus_can.SendMessage_for_timer_loop(&c620_s);
    }
}

float P(float diff)
{
    return hub_config.kp * diff;
}

float I(float diff)
{
    static float last_diff = 0;
    last_diff += diff;
    return hub_config.ki * last_diff * 0.001;
}

float D(float diff)
{
    return hub_config.kd * diff;
}

float PID(float diff) {}

void three_omni_cab() {}

void four_omni_cab()
{
    /**
     * 右上がID1, ID,4     ID,1
     * 右下がID2,
     * 左下がID3,
     * 左上がID4, ID,3     ID,2
     */

    /**
     * 前進　斜め　回転のデータを取得する。
     * 0~300 rpmまでの値を取得
     */

    /*前進*/

    for (uint16_t i = 0; i < 800; i++) {
        c620_send.value[0] = i * 10;
        c620_send.value[1] = i * 10;
        c620_send.value[2] = -i * 10;
        c620_send.value[3] = -i * 10;

        esc_bus_can.SendMessage(&c620_s);
        esc_bus_can.wait_tx_event_fin();
        /*送信完了まで待つ*/
        HAL_Delay(10);
        /*モーターの速度が更新されるであろう時間待つ*/
        /*10mSはあてずっぽう　他も同じ*/
        /*すでにHAL_Delay()で待っているためwhileで待つ必要なし*/
        if ((esc_bus_can.setup_type.buffer->nvic_.Id & 0x01) == 0x01) {
            /*ID1*/
            esc_bus_can.GetMessage(&c620_r[0]);
            c620_line_table[0][i].c620_value = c620_send.value[0];
            c620_line_table[0][i].rad_p_s    = (float)(c620_receive[0].value.rpm) / 60.0f * 2 * M_PI;
            /*radian per second = rpm / minutes * 2 * π*/
        } else {
            Error();
        }

        if ((esc_bus_can.setup_type.buffer->nvic_.Id & 0x02) == 0x02) {
            /*ID2*/
            esc_bus_can.GetMessage(&c620_r[1]);
            c620_line_table[1][i].c620_value = c620_send.value[1];
            c620_line_table[1][i].rad_p_s    = (float)(c620_receive[1].value.rpm) / 60.0f * 2 * M_PI;
        } else {
            Error();
        }

        if ((esc_bus_can.setup_type.buffer->nvic_.Id & 0x04) == 0x04) {
            /*ID3*/
            esc_bus_can.GetMessage(&c620_r[2]);
            c620_line_table[2][i].c620_value = c620_send.value[2];
            c620_line_table[2][i].rad_p_s    = (float)(c620_receive[2].value.rpm) / 60.0f * 2 * M_PI;
        } else {
            Error();
        }

        if ((esc_bus_can.setup_type.buffer->nvic_.Id & 0x08) == 0x08) {
            /*ID4*/
            esc_bus_can.GetMessage(&c620_r[3]);
            c620_line_table[3][i].c620_value = c620_send.value[3];
            c620_line_table[3][i].rad_p_s    = (float)(c620_receive[3].value.rpm) / 60.0f * 2 * M_PI;
        } else {
            Error();
        }
    }

    /*斜め*/
    for (uint16_t i = 0; i < 800; i++) {
        c620_send.value[0] = i * 10;
        // c620_send.value[1] = i * 10;
        c620_send.value[2] = -i * 10;
        // c620_send.value[3] = -i * 10;

        esc_bus_can.SendMessage(&c620_s);
        esc_bus_can.wait_tx_event_fin();
        /*送信完了まで待つ*/
        HAL_Delay(10);
        /*モーターの速度が更新されるであろう時間待つ*/

        if ((esc_bus_can.setup_type.buffer->nvic_.Id & 0x01) == 0x01) {
            /*ID1*/
            esc_bus_can.GetMessage(&c620_r[0]);
            c620_line_table[0][i].c620_value = c620_send.value[0];
            c620_line_table[0][i].rad_p_s    = (float)(c620_receive[0].value.rpm) / 60.0f * 2 * M_PI;
            /*radian per second = rpm / minutes * 2 * π*/
        } else {
            Error();
        }

        // if ((esc_bus_can.setup_type.buffer->nvic_.Id & 0x02) == 0x02) {
        //     /*ID2*/
        //     esc_bus_can.GetMessage(&c620_r[1]);
        //     c620_line_table[1][i].c620_value = c620_send.value[1];
        //     c620_line_table[1][i].rad_p_s    = (float)(c620_receive[1].value.rpm) / 60.0f * 2 * M_PI;
        // } else {
        //     Error();
        // }

        if ((esc_bus_can.setup_type.buffer->nvic_.Id & 0x04) == 0x04) {
            /*ID3*/
            esc_bus_can.GetMessage(&c620_r[2]);
            c620_line_table[2][i].c620_value = c620_send.value[2];
            c620_line_table[2][i].rad_p_s    = (float)(c620_receive[2].value.rpm) / 60.0f * 2 * M_PI;
        } else {
            Error();
        }

        // if ((esc_bus_can.setup_type.buffer->nvic_.Id & 0x08) == 0x08) {
        //     /*ID4*/
        //     esc_bus_can.GetMessage(&c620_r[3]);
        //     c620_line_table[3][i].c620_value = c620_send.value[3];
        //     c620_line_table[3][i].rad_p_s    = (float)(c620_receive[3].value.rpm) / 60.0f * 2 * M_PI;
        // } else {
        //     Error();
        // }
    }

    /*回転*/
}

void Error() {}
