
#include "app.h"

#include "../maidui3_hal/Drivers/FDCAN/mXCAN.hpp"
// #include "driver_fdcan.hpp"
//  #include "gn10_can/core/fdcan_bus.hpp"

// gn10_can::drivers::DriverSTM32FDCAN main_bus_fdcan(&hfdcan1);
// gn10_can::FDCANBus main_fdcan_bus(main_bus_fdcan);

#define maidui3_xcan       maidui3_hal::Drivers::XCAN
#define C620_bace_id       0x200
#define c620_send_id_Fside 0x200
#define c620_send_id_Lside 0x1FF

maidui3_hal::Drivers::XCAN::xcan main_bus_can(
    &hfdcan1,
    maidui3_xcan::fifo::FIFO0,
    maidui3_xcan::can_frame::Classic_CAN,
    maidui3_xcan::id_filter_type::Non_mask_id,
    0,
    0
);

maidui3_hal::Drivers::XCAN::xcan esc_bus_can(
    &hfdcan2,
    maidui3_xcan::fifo::FIFO1,
    maidui3_xcan::can_frame::Classic_CAN,
    maidui3_xcan::id_filter_type::Non_mask_id,
    0,
    0
);

maidui3_xcan::hxcan_frame c620;

union c620_send_data_frame {
    uint8_t data[8];
    int16_t value[4];
};

union c620_receive_data_frame {
    uint8_t data[8];
    struct {
        uint16_t rote_angle; /*角度*/
        uint16_t rpm;        /*速度*/
        uint8_t temperature; /*温度*/
        uint8_t unused;
    } value;
};

c620_send_data_frame c620_send;
c620_receive_data_frame c620_receive[4];

void APP_Setup()
{
    // HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
    // HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);
    // HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET);
    // HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_SET);

    HAL_Delay(10);

    esc_bus_can.set_Id(C620_bace_id + 1);
    esc_bus_can.set_Id(C620_bace_id + 2);
    esc_bus_can.set_Id(C620_bace_id + 3);
    esc_bus_can.set_Id(C620_bace_id + 4);
    if (esc_bus_can.init()) {
        HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
        while (1);
    } /*CANの初期化*/
    if (main_bus_can.init()) {
        HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
        while (1);
    }

    c620.id     = c620_send_id_Fside;
    c620.len    = 8;
    c620.data_p = c620_send.data;

    /*CAN,SPI,USBの初期化*/

    // HAL_Delay(500);

    // HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);
    // HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
    // HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_RESET);
    // HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_RESET);

    /*キャリブレーション開始*/

    c620_send.value[0] = 0x00;
    c620_send.value[1] = 0x00;
    c620_send.value[2] = 0x00;
    c620_send.value[3] = 0x00;

    if (esc_bus_can.SendMessage(&c620)) {
        HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
    } /*全ESC停止*/

    if (main_bus_can.SendMessage(&c620)) {
        HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
    }

    // while (1) {
    //     if (esc_bus_can.setup_type.callback_flag_.Id[0] && esc_bus_can.setup_type.callback_flag_.Id[1] &&
    //         esc_bus_can.setup_type.callback_flag_.Id[2] && esc_bus_can.setup_type.callback_flag_.Id[3])
    //         break;
    //     HAL_Delay(10);
    // } /*全ESCから呼ばれるまで待つ*/

    esc_bus_can.setup_type.callback_flag_.Id[0] = 0;
    esc_bus_can.setup_type.callback_flag_.Id[1] = 0;
    esc_bus_can.setup_type.callback_flag_.Id[2] = 0;
    esc_bus_can.setup_type.callback_flag_.Id[3] = 0;

    /*キャリブレーション終了*/
}

void APP_Loop()
{
    HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_SET);

    // if (esc_bus_can.SendMessage(&c620)) {
    //     HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
    // }
    // HAL_Delay(100);
    // if (main_bus_can.SendMessage(&c620)) {
    //     HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
    // }

    HAL_Delay(100);
}
