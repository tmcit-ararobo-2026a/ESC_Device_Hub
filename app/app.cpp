
#include "app.h"

#include "../maidui3_hal/Drivers/FDCAN/mXCAN.hpp"
#include "driver_fdcan.hpp"
#include "gn10_can/core/fdcan_bus.hpp"
#include "gn10_can/devices/esc_hub_config.hpp"
#include "gn10_can/devices/esc_hub_server.hpp"

gn10_can::drivers::DriverSTM32FDCAN main_bus_fdcan(&hfdcan1);
gn10_can::FDCANBus main_fdcan_bus(main_bus_fdcan);
gn10_can::devices::ESCHubServer main_bus(main_fdcan_bus, 0);

#define maidui3_xcan maidui3_hal::Drivers::XCAN
#define C620_bace_id 0x200

maidui3_xcan::xcan esc_bus_can(
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

void setup()
{
    HAL_Delay(10);
    HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);

    if (esc_bus_can.init()) {
        HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
    } /*CANの初期化*/

    c620.id_     = C620_bace_id;
    c620.len_    = 8;
    c620.data_p_ = c620_send.data;

    /*CAN,SPI,USBの初期化*/

    /*キャリブレーション開始*/

    c620_send.value[0] = 0x00;
    c620_send.value[1] = 0x00;
    c620_send.value[2] = 0x00;
    c620_send.value[3] = 0x00;

    HAL_Delay(10);

    if (esc_bus_can.SendMessage(&c620)) {
        HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
    } /*全ESC停止*/

    esc_bus_can.setup_type.callback_flag_.Id = 0;

    while (1) {
    };

    /*キャリブレーション終了*/
    HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);

    /*ループ開始*/
    HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET);
}

void loop() {}
