
#pragma once

#include "../../maidui3_hal/Drivers/FDCAN/mXCAN_manager.hpp"
#include "gn10_can/drivers/fdcan_driver_interface.hpp"
#include "main.h"

namespace gn10_can {
namespace drivers {

class DriverSTM32FDCAN : public IFDCANDriver
{
public:
    DriverSTM32FDCAN(FDCAN_HandleTypeDef* hfdcan) {}

    bool init();
    bool send(const FDCANFrame& frame) override;
    bool receive(FDCANFrame& out_frame) override;

protected:
    maidui3_hal::Drivers::XCAN::xcan main_silent_fdcan;
};
}  // namespace drivers
}  // namespace gn10_can
