
#include "app/app.h"

#include "app/driver_fdcan.hpp"
#include "gn10_can/core/fdcan_bus.hpp"
#include "maidui3_hal/Drivers/FDCAN/mXCAN.hpp"

gn10_can::drivers::DriverSTM32FDCAN main_bus_fdcan(&hfdcan1);
gn10_can::FDCANBus main_fdcan_bus(main_bus_fdcan);

#define maidui3_xcan       maidui3_hal::Drivers::XCAN
#define C620_bace_id       0x200
#define c620_send_id_Fside 0x200
#define c620_send_id_Lside 0x1FF

maidui3_hal::Drivers::XCAN::xcan esc_bus_can(
    &hfdcan2,
    maidui3_xcan::fifo::FIFO1,
    maidui3_xcan::can_frame::Classic_CAN,
    maidui3_xcan::id_filter_type::mask_four_id,
    2000,
    1
);

maidui3_hal::Drivers::XCAN::hxcan_frame c620;

union c620_data_frame {
    uint8_t data[8];
    int16_t value[4];
};

c620_data_frame c620_data;

void APP_Setup()
{
    HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_SET);

    esc_bus_can.set_Id(C620_bace_id + 1);
    esc_bus_can.set_Id(C620_bace_id + 2);
    esc_bus_can.set_Id(C620_bace_id + 3);
    esc_bus_can.set_Id(C620_bace_id + 4);
    if (esc_bus_can.init()) {
        while (1);
    }

    c620.id     = c620_send_id_Fside;
    c620.len    = 8;
    c620.data_p = c620_data.data;
    esc_bus_can.SendMessage(c620);

    HAL_Delay(500);

    HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_RESET);
}

void APP_Loop() {}
