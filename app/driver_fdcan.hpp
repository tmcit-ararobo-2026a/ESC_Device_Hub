
#pragma once

#include "../../maidui3_hal/Drivers/FDCAN/mXCAN.hpp"
#include "gn10_can/core/fdcan_bus.hpp"
#include "gn10_can/drivers/fdcan_driver_interface.hpp"
#include "main.h"

#define maidui3_xcan maidui3_hal::Drivers::XCAN

maidui3_xcan::xcan main_silent_fdcan(
    NULL,
    maidui3_xcan::fifo::FIFO0,
    maidui3_xcan::can_frame::FDCAN,
    maidui3_xcan::id_filter_type::Non_mask_id,
    0,
    0
);

namespace gn10_can {
namespace drivers {

class DriverSTM32FDCAN : public IFDCANDriver
{
public:
    FDCAN_RxHeaderTypeDef gn10_FDCAN_RxHeader;
    maidui3_xcan::hxcan_frame data_frame;
    uint8_t data[64];

public:
    DriverSTM32FDCAN(FDCAN_HandleTypeDef* hfdcan)
    {
        main_silent_fdcan.set_FDCAN_HandleTypedef(hfdcan);
    }

    bool init();
    bool send(const FDCANFrame& frame) override;
    bool receive(FDCANFrame& out_frame) override;
};
}  // namespace drivers
}  // namespace gn10_can

extern gn10_can::drivers::DriverSTM32FDCAN main_bus_fdcan;
extern gn10_can::FDCANBus main_fdcan_bus;
