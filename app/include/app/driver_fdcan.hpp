
#pragma once

#include "../../maidui3_hal/Drivers/FDCAN/mXCAN_manager.hpp"
#include "gn10_can/drivers/can_driver_interface.hpp"
#include "main.h"

namespace gn10_can {
namespace drivers {

class DriverSTM32FDCAN : public ICANDriver
{
public:
    DriverSTM32FDCAN(FDCAN_HandleTypeDef* hfdcan)
    {
        maidui3_hal::Drivers::XCAN::xcan_manager.xcan_init(
            hfdcan,
            maidui3_hal::Drivers::XCAN::fifo::FIFO0,
            maidui3_hal::Drivers::XCAN::can_frame::FDCAN,
            maidui3_hal::Drivers::XCAN::id_filter_type::Non_id
        );
    }

    bool init();
    bool send(const CANFrame& frame) override;
    bool receive(CANFrame& out_frame) override;
};
}  // namespace drivers
}  // namespace gn10_can
