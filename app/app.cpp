
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
    maidui3_xcan::id_filter_type::mask_four_id,
    0,
    0
);

maidui3_xcan::hxcan_frame c620;

union c620_send_data_frame {
    uint8_t data[8];
    int16_t value[4];  // -16384 ~ 16384
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

struct c620_speed_box {
    float rad_p_s;
    uint16_t c620_value;
};

c620_speed_box c620_speed_table[4][164];

void setup()
{
    HAL_Delay(10);
    HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);

    esc_bus_can.set_Id(C620_bace_id + 1);
    esc_bus_can.set_Id(C620_bace_id + 2);
    esc_bus_can.set_Id(C620_bace_id + 3);
    esc_bus_can.set_Id(C620_bace_id + 4);

    if (esc_bus_can.init()) {
        HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
    } /*CANの初期化*/

    c620.id_     = C620_bace_id;
    c620.len_    = 0x08;
    c620.data_p_ = c620_send.data;

    esc_bus_can.setup_type.callback_flag_.Id = 0;
    esc_bus_can.buffer.nvic_.Id              = 0;
    esc_bus_can.buffer.nvic_.Rx_Callback     = 0;
    esc_bus_can.buffer.nvic_.Rx_Timeout      = 0;
    esc_bus_can.buffer.nvic_.Tx_Callback     = 0;

    /*CAN,SPI,USBの初期化*/

    c620_send.value[0] = 0x00;
    c620_send.value[1] = 0x00;
    c620_send.value[2] = 0x00;
    c620_send.value[3] = 0x00;

    if (esc_bus_can.SendMessage(&c620)) {
        HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
    } /*全ESC停止*/
    HAL_Delay(10);

    /*キャリブレーション開始*/

    /**
     * 右上がID1,右下がID2,左下がID3,左上がID4
     * キャリブレーション中の増加量は値で100
     */

    // for (uint16_t i = 0; i < 164; i++) {
    //     c620_send.value[0] = i * 100;
    //     c620_send.value[1] = i * 100;
    //     c620_send.value[2] = i * 100;
    //     c620_send.value[3] = i * 100;
    //     esc_bus_can.SendMessage(&c620);
    //     while (!esc_bus_can.buffer.nvic_.Rx_Callback) {
    //         esc_bus_can.GetMessage(&c620);
    //     }
    // }

    /*キャリブレーション終了*/
    HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);

    /*ループ開始*/
    HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET);
}

void loop()
{
    if (esc_bus_can.buffer.nvic_.Rx_Callback) {
        esc_bus_can.buffer.nvic_.Rx_Callback = 0;
    }
}
