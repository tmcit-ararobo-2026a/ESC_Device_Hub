
#include "app/driver_fdcan.hpp"

namespace gn10_can {
namespace drivers {

bool DriverSTM32FDCAN::init() {}

bool DriverSTM32FDCAN::send(const FDCANFrame& frame) {}

bool DriverSTM32FDCAN::receive(FDCANFrame& out_frame) {}

}  // namespace drivers
}  // namespace gn10_can