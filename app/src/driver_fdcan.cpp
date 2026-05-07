
#include "app/driver_fdcan.hpp"

namespace gn10_can {
namespace drivers {

bool DriverSTM32FDCAN::init() {}

bool DriverSTM32FDCAN::send(const CANFrame& frame) {}

bool DriverSTM32FDCAN::receive(CANFrame& out_frame) {}

}  // namespace drivers
}  // namespace gn10_can